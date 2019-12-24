/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/**
 * Shuts down the Jetson. <b>Use with extreme care!!</b>
 */
public class ShutdownJetson extends InstantCommand {

    public ShutdownJetson() {
        addRequirements(Robot.vision);
    }

    @Override
    public boolean runsWhenDisabled() {
        // this command runs when the robot is disabled
        return true;
    }

    // Called once when the command executes
    @Override
    public void initialize() {
        if(!Robot.vision.ready()) {
            Robot.logger.logWarning("Attempting to shutdown the Jetson, but it is not up!");
            return;
        }
        Robot.vision.shutdownJetson();
        Robot.visionStatusEntry.setBoolean(false);
        Robot.logger.logInfoFine("Jetson has been shutdown");
    }
}
