/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.BeautifulRobot;

/**
 * Runs Essie to intake cargo until the sensor inside it is activated.
 * Note that if there is no cargo, this command will never terminate.
 */
public class AutoCargoIntake extends Command {
    public AutoCargoIntake() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.essie);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.essie.startIntake();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // When this command finishes, rumble the controller and flash the LEDs
        if(Robot.essie.hasCargo()) {
            OI.pickupRumble.execute();
            @SuppressWarnings("resource")
            Command pulse = new PulseBeautifulRobot(1.5, 10, BeautifulRobot.Color.fromAlliance(DriverStation.getInstance().getAlliance()));
            pulse.start();
            return true;
        }
        else {
            return false;
        }
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
