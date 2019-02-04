/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Vision.VisionException;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class AdvancedVisionAlign extends Command {

    private boolean error = false;

    private final long RESPONSE_TIMEOUT = 200; // Milliseconds
    private final double RECALCULATION_INTERVAL = 0.5; // Seconds

    private TrajectoryParams params;
    private TankDriveTrajectory trajectory;
    
    private FollowTrajectory followerCommand;

    private double lastTime;

    public AdvancedVisionAlign() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // First check if vision is ready
        if(!Robot.vision.ready()) {
            SmartDashboard.putString("Last Error", "Error: Vision is offline");
            OI.errorRumbleDriverMinor.execute();
            error = true;
            return;
        }
        // If vision is not enabled, attempt to enable it
        if(!Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(true, true, 100);
                // Short delay to wait for camera to switch
                Thread.sleep(100);
            }
            catch(VisionException | InterruptedException e) {
                SmartDashboard.putString("Last Error", "Error: " + e.getMessage());
                OI.errorRumbleDriverMinor.execute();
                error = true;
                return;
            }
        }

        // Get the parameters we need
        double visionAngleOffset, visionXOffset, visionYOffset;
        try {
            long start = System.currentTimeMillis();
            // Give the Jetson some time to figure it out
            while(Double.isNaN(visionAngleOffset = Robot.vision.getTargetAngleOffset())) {
                try {
                    // If we exceeded the time limit, signal an error
                    if(System.currentTimeMillis() - start >= RESPONSE_TIMEOUT) {
                        SmartDashboard.putString("Last Error", "Error: Could not find vision target");
                        OI.errorRumbleDriverMinor.execute();
                        error = true;
                        return;
                    }

                    // Sleep for 20ms
                    Thread.sleep(20);
                }
                catch(InterruptedException e) {
                    SmartDashboard.putString("Last Error", "Error: That wasn't supposed to happen");
                    OI.errorRumbleDriverMinor.execute();
                    error = true;
                    return;
                }
            }
            visionXOffset = Robot.vision.getTargetXOffset();
            visionYOffset = Robot.vision.getTargetYOffset();
        }
        catch(VisionException e) {
            SmartDashboard.putString("Last Error", "Error: Vision went offline unexpectedly");
            OI.errorRumbleDriverMajor.execute();
            error = true;
            return;
        }

        params = new TrajectoryParams();
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        params.segmentCount = 100;
        // Set the waypoints
        params.waypoints = new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            // The second waypoint has coordinates relative to the first waypoint, which is just the robot's current position
            new Waypoint(visionXOffset, visionYOffset, -visionAngleOffset + Math.PI / 2),
        };
        // Set alpha to be 3/4 of the diagonal distance
        params.alpha = Math.sqrt(visionXOffset * visionXOffset + visionYOffset * visionYOffset) * 0.75;

        trajectory = new TankDriveTrajectory(RobotMap.specs, params);
        followerCommand = new FollowTrajectory(trajectory);
        // We can't call start() on the command as it also requires drivetrain, which would cause this command to be interrupted. 
        // Thus we just call the raw methods and not hand control to WPILib.
        followerCommand.initialize();

        lastTime = timeSinceInitialized();
    }

    // For concurrent trajectory generation
    private final ExecutorService executorService = Executors.newSingleThreadExecutor();
    private Future<TankDriveTrajectory> trajGenFuture;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        followerCommand.execute();
        // Check the result of the asynchronous computation
        if(trajGenFuture != null && trajGenFuture.isDone()) {
            // Simply replace the follower command
            try {
                followerCommand = new FollowTrajectory(trajGenFuture.get());
                // Don't forget this
                followerCommand.initialize();
            }
            catch(CancellationException e) {
                // Well, it seems like that the computation was so slow that it got cancelled. 
            }
            catch(InterruptedException | ExecutionException e) {
                // This should not happen.
                SmartDashboard.putString("Last Error", "Error: Unexpected exception in asynchronous trajectory recalculation");
            }
        }
        if(timeSinceInitialized() - lastTime >= RECALCULATION_INTERVAL) {
            // Recalculate the trajectory, if the target is still in sight
            double angleOffset, xOffset, yOffset;
            try {
                angleOffset = Robot.vision.getTargetAngleOffset();
                xOffset = Robot.vision.getTargetXOffset();
                yOffset = Robot.vision.getTargetYOffset();
            }
            catch(VisionException e) {
                // Report the error, but don't cause the current command to finish
                SmartDashboard.putString("Last Error", "Error: Vision went offline unexpectedly");
                lastTime = timeSinceInitialized();
                return;
            }
            // Make sure it's not NaN
            if(!Double.isNaN(angleOffset)) {
                // Terminate the previous trajectory generation thread if it is still not done
                if(trajGenFuture != null && !trajGenFuture.isDone()) {
                    trajGenFuture.cancel(true);
                }
                // Do the trajectory generation in a separate thread
                trajGenFuture = executorService.submit(() -> {
                    // Set the waypoints
                    params.waypoints = new Waypoint[] {
                        // The first waypoint has the velocity equal to our current velocity
                        new WaypointEx(0, 0, Math.PI / 2, (Robot.drivetrain.getLeftSpeed() + Robot.drivetrain.getRightSpeed()) / 2),
                        // The second waypoint has coordinates relative to the first waypoint, which is just the robot's current position
                        new Waypoint(xOffset, yOffset, -angleOffset + Math.PI / 2),
                    };
                    // Set alpha to be 3/4 of the diagonal distance
                    params.alpha = Math.sqrt(xOffset * xOffset + yOffset * yOffset) * 0.75;
                    return new TankDriveTrajectory(RobotMap.specs, params);
                });
                // Only update the last timestamp if the vision processing was successful
                lastTime = timeSinceInitialized();
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(error) {
            return true;
        }
        return followerCommand.isFinished();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        followerCommand.end();
        if(Robot.vision.ready() && Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(false);
            }
            catch(VisionException e) {
                SmartDashboard.putString("Last Error", "Failed to disable vision!");
                OI.errorRumbleDriverMinor.execute();
            }
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        followerCommand.interrupted();
        if(Robot.vision.ready() && Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(false);
            }
            catch(VisionException e) {
                SmartDashboard.putString("Last Error", "Failed to disable vision!");
                OI.errorRumbleDriverMinor.execute();
            }
        }
    }
}
