/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class RestartVisionServer extends InstantCommand {
    public RestartVisionServer() {
        addRequirements(Robot.vision);
    }

    @Override
    public boolean runsWhenDisabled() {
        // This command runs when the robot is disabled.
        return true;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        if(!Robot.vision.ready()) {
            Robot.logger.logWarning("Attempting to restart vision server, but vision is not up!");
            return;
        }
        Robot.vision.restartServer();
        Robot.logger.logInfoFine("Vision server restarted");
    }
}
