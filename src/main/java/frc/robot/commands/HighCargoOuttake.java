/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Runs Essie so that the motors spin to shoot the cargo through the high exit (Cargo ship).
 * 
 * Note that this command will never terminate!!
 */
public class HighCargoOuttake extends Command {
    public HighCargoOuttake() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.essie);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.essie.startOuttakeHigh();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
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
