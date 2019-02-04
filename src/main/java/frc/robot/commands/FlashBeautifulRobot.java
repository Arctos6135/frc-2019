/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;
import frc.robot.misc.BeautifulRobotDriver;
import frc.robot.subsystems.BeautifulRobot;

public class FlashBeautifulRobot extends TimedCommand {

    private final int speed;

    public FlashBeautifulRobot(double timeout, int speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super(timeout);
        requires(Robot.beautifulRobot);

        this.speed = speed;
    }

    private BeautifulRobot.Pattern pattern;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        pattern = Robot.beautifulRobot.getPattern();
        Robot.beautifulRobot.setPattern(BeautifulRobot.Pattern.PULSATING);
        Robot.beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_HIGH, (byte) speed);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_HIGH, (byte) 1);
        Robot.beautifulRobot.setPattern(pattern);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
