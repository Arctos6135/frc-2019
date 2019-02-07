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

public class EssieDefault extends Command {

    private static final double THRESHOLD = 0.6;

    public EssieDefault() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.essie);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double val = OI.operatorController.getRawAxis(OI.Controls.ESSIE_OUTTAKE);
        
        if(val >= THRESHOLD) {
            Robot.essie.startOuttakeLow();
        }
        else if(val <= -THRESHOLD) {
            Robot.essie.startOuttakeHigh();
        }
        else {
            if(OI.operatorController.getRawButton(OI.Controls.ESSIE_REVERSE_INTAKE)) {
                Robot.essie.reverseIntake();
                return;
            }

            boolean lb = OI.operatorController.getRawButton(OI.Controls.ESSIE_OUTTAKE_LOW);
            boolean hb = OI.operatorController.getRawButton(OI.Controls.ESSIE_OUTTAKE_HIGH);

            if(lb && hb || !lb && !hb) {
                Robot.essie.stop();
                return;
            }
            else if(lb) {
                Robot.essie.startOuttakeLow();
            }
            else {
                Robot.essie.startOuttakeHigh();
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.essie.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.essie.stop();
    }
}