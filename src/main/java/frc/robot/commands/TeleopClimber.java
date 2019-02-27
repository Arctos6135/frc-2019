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

public class TeleopClimber extends Command {
    public TeleopClimber() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.theL);
    }

    static final double DEADZONE = 0.15;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double up = OI.driverController.getRawAxis(OI.Controls.THE_L_UP);
        double down = OI.driverController.getRawAxis(OI.Controls.THE_L_DOWN);
        // These values will never be negative
        up = up > DEADZONE ? up : 0;
        down = down > DEADZONE ? down : 0;
        
        if(up != 0 && down != 0) {
            return;
        }
        else if(up != 0) {
            Robot.theL.set(up);
        }
        else if(down != 0) {
            Robot.theL.set(-down);
        }
        else {
            Robot.theL.set(0);
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
