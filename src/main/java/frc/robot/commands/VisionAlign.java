/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Vision.VisionException;

public class VisionAlign extends Command {
    public VisionAlign() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        requires(Robot.vision);
    }

    private boolean error = false;

    private final long RESPONSE_TIMEOUT = 600;

    // The type of this command can be changed later
    private RotateToAngle turningCommand;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        RobotLogger.logInfoFine("Basic vision align started");
        error = false;
        // Check that vision is ready
        if(!Robot.vision.ready()) {
            error = true; // Signal an error
            RobotLogger.logError("Vision is offline");
            OI.errorRumbleDriverMajor.execute();
            return;
        }
        // If vision is not on, turn on vision
        if(!Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(true, true, 100);
            }
            catch(VisionException e) {
                error = true;
                RobotLogger.logError("Vision enable failed");
                OI.errorRumbleDriverMajor.execute();
                return;
            }
        }
        
        // Get the parameters we need
        double visionResult = Double.NaN;
        try {
            long start = System.currentTimeMillis();
            // Give the Jetson some time to figure it out
            while(Double.isNaN(visionResult = Robot.vision.getTargetAngleOffset())) {
                try {
                    // If we exceeded the time limit, signal an error
                    if(System.currentTimeMillis() - start >= RESPONSE_TIMEOUT) {
                        RobotLogger.logWarning("Could not find vision target");
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
        }
        catch(VisionException e) {
            RobotLogger.logError("Vision went offline unexpectedly");
            OI.errorRumbleDriverMajor.execute();
            error = true;
            return;
        }

        // Alright, now we know that we didn't screw up!
        // With the vision code, a negative value means that the angle is towards the left
        turningCommand = new RotateToAngle(Math.abs(visionResult), visionResult < 0 ? RotateToAngle.Direction.LEFT : RotateToAngle.Direction.RIGHT);
        turningCommand.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(turningCommand != null)
            turningCommand.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(error) {
            return true;
        }
        if(turningCommand != null) {
            return turningCommand.isFinished();
        }
        else {
            return true;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        if(turningCommand != null) {
            turningCommand.end();
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
        RobotLogger.logInfoFine("Basic vision align ended");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        if(turningCommand != null) {
            turningCommand.interrupted();
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
        RobotLogger.logInfoFine("Basic vision align interrupted");
    }
}
