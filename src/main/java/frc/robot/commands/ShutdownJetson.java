/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Shuts down the Jetson. <b>Use with extreme care!!</b>
 */
public class ShutdownJetson extends InstantCommand {

    public ShutdownJetson() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.vision);

        // By default, a command will not run when the robot is disabled
        // This allows the ShutdownJetson command to run anytime
        setRunWhenDisabled(true);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        if(!Robot.vision.ready()) {
            Robot.logger.logWarning("Attempting to shutdown the Jetson, but it is not up!");
            return;
        }
        Robot.vision.shutdownJetson();
        Robot.visionStatusEntry.setBoolean(false);
        Robot.logger.logInfoFine("Jetson has been shutdown");
    }
}
