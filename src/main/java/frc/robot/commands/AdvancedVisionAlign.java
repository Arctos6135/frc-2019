/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;
//import java.util.concurrent.Future;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Vision.VisionException;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class AdvancedVisionAlign extends Command {

    private boolean error = false;

    private final long RESPONSE_TIMEOUT = 600; // Milliseconds

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
        RobotLogger.logInfoFine("Advanced vision align started");
        error = false;
        // First check if vision is ready
        if(!Robot.vision.ready()) {
            RobotLogger.logError("Vision is offline");
            OI.errorRumbleDriverMinor.execute();
            error = true;
            return;
        }
        // If vision is not enabled, attempt to enable it
        if(!Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(true, true, 300);
                Thread.sleep(400);
            }
            catch(VisionException | InterruptedException e) {
                RobotLogger.logError(e.getMessage());
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
                        RobotLogger.logError("Could not find vision target");
                        OI.errorRumbleDriverMinor.execute();
                        error = true;
                        return;
                    }

                    // Sleep for 20ms
                    Thread.sleep(20);
                }
                catch(InterruptedException e) {
                    RobotLogger.logError("Unexpected InterruptedException");
                    OI.errorRumbleDriverMinor.execute();
                    error = true;
                    return;
                }
            }
            visionXOffset = Robot.vision.getTargetXOffset();
            visionYOffset = Robot.vision.getTargetYOffset();
        }
        catch(VisionException e) {
            RobotLogger.logError("Vision went offline unexpectedly");
            OI.errorRumbleDriverMajor.execute();
            error = true;
            return;
        }

        if(visionYOffset <= 3) {
            RobotLogger.logError("Not enough distance to vision target!");
            OI.errorRumbleDriverMinor.execute();
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
            new Waypoint(visionXOffset, visionYOffset, Math.toRadians(-visionAngleOffset) + Math.PI / 2),
        };
        // Set alpha to be 3/4 of the diagonal distance
        params.alpha = Math.sqrt(visionXOffset * visionXOffset + visionYOffset * visionYOffset) * 0.75;
        
        trajectory = new TankDriveTrajectory(FollowTrajectory.getSpecs(), params);
        followerCommand = new FollowTrajectory(trajectory);
        // We can't call start() on the command as it also requires drivetrain, which would cause this command to be interrupted. 
        // Thus we just call the raw methods and not hand control to WPILib.
        followerCommand.initialize();
        
        if(Robot.isInDebugMode) {
            SmartDashboard.putNumber("Auto Align X Offset", visionXOffset);
            SmartDashboard.putNumber("Auto Align Y Offset", visionYOffset);
            SmartDashboard.putNumber("Auto Align Angle Offset", visionAngleOffset);

            RobotLogger.logInfoFiner("Auto align X offset: " + visionXOffset);
            RobotLogger.logInfoFiner("Auto align Y offset: " + visionYOffset);
            RobotLogger.logInfoFiner("Auto align angle offset: " + visionAngleOffset);
        }
    }
    
    // For concurrent trajectory generation
    //private final ExecutorService executorService = Executors.newSingleThreadExecutor();
    //private Future<TankDriveTrajectory> trajGenFuture;
    
    double visionXOffset = Double.NaN, visionYOffset = Double.NaN, visionAngleOffset = Double.NaN;
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
        /*try {
            // Try to get the vision values
            // If this throws and error, the async trajectory generation is skipped
            visionAngleOffset = Robot.vision.getTargetAngleOffset();
            visionXOffset = Robot.vision.getTargetXOffset();
            visionYOffset = Robot.vision.getTargetYOffset();
            // Output them if in debug mode
            if(Robot.isInDebugMode) {
                SmartDashboard.putNumber("Auto Align X Offset", visionXOffset);
                SmartDashboard.putNumber("Auto Align Y Offset", visionYOffset);
                SmartDashboard.putNumber("Auto Align Angle Offset", visionAngleOffset);
            }

            // Check if the async trajectory generation is done
            if(trajGenFuture != null && trajGenFuture.isDone()) {
                try {
                    // If it's done, replace the current trajectory
                    trajectory = trajGenFuture.get();
                    // Replace the follower command and initialize
                    followerCommand = new FollowTrajectory(trajectory);
                    followerCommand.initialize();
                }
                // Catch a crap ton of exceptions
                catch(InterruptedException e) {
                    RobotLogger.logError("New trajectory generation was interrupted");
                }
                catch(ExecutionException e) {
                    RobotLogger.logError("Exception in new trajectory generation: " + e.getMessage());
                }
                catch(CancellationException e) {
                    RobotLogger.logError("New trajectory generation was cancelled");
                }
                
                // Start a new generation thread
                // Make sure that the target is in sight and more than 10 inches away
                if(!Double.isNaN(visionAngleOffset) && visionYOffset > 10) {
                    // Submit a callable that generates the new trajectory
                    trajGenFuture = executorService.submit(() -> {
                        params = new TrajectoryParams();
                        params.waypoints = new Waypoint[] {
                            new WaypointEx(0, 0, Math.PI / 2, (Robot.drivetrain.getLeftSpeed() + Robot.drivetrain.getRightSpeed()) / 2),
                            new Waypoint(visionXOffset, visionYOffset, Math.toRadians(-visionAngleOffset) + Math.PI / 2),
                        };
                        params.alpha = params.alpha = Math.sqrt(visionXOffset * visionXOffset + visionYOffset * visionYOffset) * 0.75;
                        params.isTank = true;
                        params.segmentCount = 100;
                        return new TankDriveTrajectory(RobotMap.specs, params);
                    });
                }
                // If conditions are not met, set the future to be null to attempt generation again next loop
                else {
                    trajGenFuture = null;
                }
            }
            // Otherwise if the future is null then generate it
            else if(trajGenFuture == null) {
                if(!Double.isNaN(visionAngleOffset) && visionYOffset > 10) {
                    trajGenFuture = executorService.submit(() -> {
                        params = new TrajectoryParams();
                        params.waypoints = new Waypoint[] {
                            new WaypointEx(0, 0, Math.PI / 2, (Robot.drivetrain.getLeftSpeed() + Robot.drivetrain.getRightSpeed()) / 2),
                            new Waypoint(visionXOffset, visionYOffset, Math.toRadians(-visionAngleOffset) + Math.PI / 2),
                        };
                        params.alpha = params.alpha = Math.sqrt(visionXOffset * visionXOffset + visionYOffset * visionYOffset) * 0.75;
                        params.isTank = true;
                        params.segmentCount = 100;
                        return new TankDriveTrajectory(RobotMap.specs, params);
                    });
                }
                else {
                    trajGenFuture = null;
                }
            }
        }
        catch(VisionException e) {
            // Report the error, but don't cause the current command to finish
            RobotLogger.logError("Vision went offline unexpectedly");
        }*/

        // Execute the follower for one cycle
        if(followerCommand != null) {
            followerCommand.execute();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(error) {
            RobotLogger.logWarning("Advanced vision align encountered an error");
            return true;
        }
        return followerCommand.isFinished() /*|| visionYOffset < 20*/;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        if(followerCommand != null) {
            followerCommand.end();
        }
        if(Robot.vision.ready() && Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(false);
            }
            catch(VisionException e) {
                RobotLogger.logError("Failed to disable vision");
                OI.errorRumbleDriverMinor.execute();
            }
        }
        RobotLogger.logInfoFine("Advanced vision align ended");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        if(followerCommand != null) {
            followerCommand.interrupted();
        }
        if(Robot.vision.ready() && Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(false);
            }
            catch(VisionException e) {
                RobotLogger.logError("Failed to disable vision");
                OI.errorRumbleDriverMinor.execute();
            }
        }
        RobotLogger.logInfoFine("Advanced vision align interrupted");
    }
}
