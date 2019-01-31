/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Vision.VisionException;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class AdvancedVisionAlign extends Command {

    private boolean error = false;

    private final long RESPONSE_TIMEOUT = 200; // Milliseconds

    private TrajectoryParams params;
    private TankDriveTrajectory trajectory;
    
    private FollowTrajectory followerCommand;

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
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        followerCommand.execute();
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
