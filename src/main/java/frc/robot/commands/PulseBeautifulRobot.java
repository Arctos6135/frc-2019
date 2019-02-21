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
import frc.robot.misc.RobotLogger;

public class PulseBeautifulRobot extends TimedCommand {

    private final int speed;
    private final BeautifulRobotDriver.Color color;

    public PulseBeautifulRobot(double timeout, int speed, BeautifulRobotDriver.Color color) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super(timeout);
        requires(Robot.beautifulRobot);

        this.speed = speed;
        this.color = color;
    }

    private BeautifulRobotDriver.Pattern pattern;
    private BeautifulRobotDriver.Color initColor;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        RobotLogger.logInfoFine("PulseBeautifulRobot started");
        pattern = Robot.beautifulRobot.getPattern();
        initColor = Robot.beautifulRobot.getColor();
        if(pattern != BeautifulRobotDriver.Pattern.PULSATING) {
            Robot.beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.PULSATING);
        }
        if(initColor != color) {
            Robot.beautifulRobot.setColor(color);
        }
        Robot.beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_HIGH, (byte) speed);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        RobotLogger.logInfoFine("PulseBeautifulRobot ended");
        Robot.beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_HIGH, (byte) 1);
        if(initColor != color) {
            Robot.beautifulRobot.setColor(initColor);
        }
        Robot.beautifulRobot.setPattern(pattern);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
