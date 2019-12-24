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

/**
 * Runs Essie to intake cargo until the sensor inside it is activated.
 * Note that if there is no cargo, this command will never terminate.
 */
public class AutoCargoIntake extends CommandBase {
    public AutoCargoIntake() {
        addRequirements(Robot.essie);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Robot.logger.logInfoFine("Essie autointake started");
        Robot.essie.startIntake();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // When this command finishes, rumble the controller and flash the LEDs
        if(Robot.essie.hasCargo()) {
            OI.pickupRumbleOperator.execute();
            OI.pickupRumbleDriver.execute();
            Robot.logger.logInfoFine("Essie autopickup ended");
            return true;
        }
        else {
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Robot.essie.stop();
    }
}
