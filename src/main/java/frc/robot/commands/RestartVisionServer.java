/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;

public class RestartVisionServer extends InstantCommand {
    public RestartVisionServer() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.vision);

        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if(!Robot.vision.ready()) {
            RobotLogger.logWarning("Attempting to restart vision server, but vision is not up!");
            return;
        }
        Robot.vision.restartServer();
        RobotLogger.logInfoFine("Vision server restarted");
    }
}
