/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class EssieDefault extends CommandBase {

    private static final double THRESHOLD = 0.6;

    public EssieDefault() {
        addRequirements(Robot.essie);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double val = OI.operatorController.getRawAxis(OI.Controls.ESSIE_OUTTAKE);
        
        if(val >= THRESHOLD) {
            Robot.essie.startOuttakeLow();
        }
        else if(val <= -THRESHOLD) {
            Robot.essie.startOuttakeHigh();
        }
        else {
            val = OI.operatorController.getRawAxis(OI.Controls.ESSIE_INTAKE);
            if(val >= THRESHOLD) {
                Robot.essie.startIntakeFromMiddle();
            }
            else if(val <= -THRESHOLD) {
                Robot.essie.reverseIntake();
            }
            else {
                Robot.essie.stop();
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Robot.essie.stop();
    }
}
