/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.misc.BeautifulRobotDriver;
import frc.robot.misc.RobotLogger;

/**
 * Flashes the BeautifulRobot&#8482; a number of times.
 */
public class FlashBeautifulRobot extends Command {

    private final BeautifulRobotDriver.Color color;
    private final int duration;
    private int count;
    private final int initCount;

    private long last;

    /**
     * Constructor.
     * @param color The colour to flash
     * @param duration The time the LEDs stay on/off
     * @param count The number of flashes
     */
    public FlashBeautifulRobot(BeautifulRobotDriver.Color color, int duration, int count) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.beautifulRobot);

        this.color = color;
        this.duration = duration;
        this.initCount = count;
    }

    private BeautifulRobotDriver.Pattern initMode;
    private BeautifulRobotDriver.Color initColor;
    private boolean initState;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        RobotLogger.logInfoFine("FlashBeautifulRobot started");
        count = initCount;
        initMode = Robot.beautifulRobot.getPattern();
        initColor = Robot.beautifulRobot.getColor();
        initState = Robot.beautifulRobot.isOn();

        // Set mode, colour and turn on
        if(initMode != BeautifulRobotDriver.Pattern.SOLID) {
            Robot.beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.SOLID);
        }
        if(initColor != color) {
            Robot.beautifulRobot.setColor(color);
        }
        if(!initState) {
            Robot.beautifulRobot.turnOn();
        }
        last = System.currentTimeMillis();
    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        System.out.println(count);
        // If the duration has elapsed, toggle the on/off state of the LEDs
        if(System.currentTimeMillis() - last >= duration) {
            if(Robot.beautifulRobot.isOn()) {
                Robot.beautifulRobot.turnOff();
            }
            else {
                Robot.beautifulRobot.turnOn();
                // For each off-then-on cycle, decrement the counter
                count --;
            }
            last = System.currentTimeMillis();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // When the counter reaches 0 this command is finished
        return count <= 0;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        RobotLogger.logInfoFine("FlashBeautifulRobot ended");
        // Reset the mode, colour and on/off state if they were changed
        if(initMode != BeautifulRobotDriver.Pattern.SOLID) {
            Robot.beautifulRobot.setPattern(initMode);
        }
        if(initColor != color) {
            Robot.beautifulRobot.setColor(initColor);
        }
        if(initState != Robot.beautifulRobot.isOn()) {
            if(initState) {
                Robot.beautifulRobot.turnOn();
            }
            else {
                Robot.beautifulRobot.turnOff();
            }
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
