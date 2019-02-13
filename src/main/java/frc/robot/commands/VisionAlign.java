/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.VisionException;

public class VisionAlign extends Command {
    public VisionAlign() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        requires(Robot.vision);
    }

    private boolean error = false;

    // The type of this command can be changed later
    private RotateToAngle turningCommand;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // Check that vision is ready
        if(!Robot.vision.ready()) {
            error = true; // Signal an error
            Robot.error("Vision is offline");
            return;
        }
        // If vision is not on, turn on vision
        if(!Robot.vision.getVisionEnabled()) {
            try {
                Robot.vision.setVisionEnabled(true, true, 100);
            }
            catch(VisionException e) {
                error = true;
                Robot.error("Vision enable failed");
                return;
            }
        }
        
        double visionResult = 0;
        try {
            // Give the Jetson up to 300ms to figure the vision stuff out
            int sleepCount = 0;
            while(Double.isNaN(visionResult = Robot.vision.getVisionResult())) {
                try {
                    Thread.sleep(10);
                    sleepCount ++;
                    if (sleepCount >= 30) {
                        error = true;
                        Robot.error("Vision target not found");
                        return;
                    }
                }
                catch(InterruptedException e) {
                    Robot.error("Unexpected InterruptedException");
                }
            }
        }
        catch(VisionException e) {
            // This message should never appear. Contact your system administrator if you see this.
            Robot.error("LOL XD You're screwed");
        }

        // Alright, now we know that we didn't screw up!
        // With the vision code, a negative value means that the angle is towards the left
        turningCommand = new RotateToAngle(Math.abs(visionResult), visionResult < 0 ? RotateToAngle.Direction.LEFT : RotateToAngle.Direction.RIGHT);
        turningCommand.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(error) {
            return true;
        }
        return turningCommand.isFinished();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
