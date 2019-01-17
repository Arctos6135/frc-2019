/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.misc.BeautifulRobotDriver.Operation;

/**
 * BeautifulRobot&#8482; WS2812 RGB LED Strip Controller Subsystem.
 */
public class BeautifulRobot extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    // Controls whether the strip is enabled at all
    private boolean enabled = false;
    /**
     * Gets whether the BeautifulRobot&#8482; is enabled.
     * By default, the BeautifulRobot&#8482; is disabled and all operations have no effect.
     * 
     * @return Whether the BeautifulRobot&#8482; is enabled
     */
    public boolean getEnabled() {
        return enabled;
    }
    /**
     * Sets whether the BeautifulRobot&#8482; is enabled.
     * By default, the BeautifulRobot&#8482; is disabled and all operations have no effect.
     * 
     * @param enabled Whether the BeautifulRobot&#8482; is enabled
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Writes a raw command to the BeautifulRobot&#8482;.
     * @param op The operation
     * @param param The parameter
     */
    public void writeCommand(Operation op, byte param) {
        if(enabled) {
            RobotMap.beautifulRobotDriver.concurrentWriteWithCheck(op, param, 3);
        }
    }
    
    /**
     * Resets the BeautifulRobot&#8482;.
     */
    public void reset() {
        // Send reset signal
        writeCommand(Operation.RESET, (byte) 0);
        init();
    }
    /**
     * Initializes the BeautifulRobot&#8482;. This function is called automatically after {@link #reset()}
     * and construction.
     */
    public void init() {
        // Set brightness to 20%
        writeCommand(Operation.BRIGHTNESS, (byte) 20);
        // Set mode to pulsating colour
        writeCommand(Operation.MODE, (byte) 1);
    }
    /**
     * Sets the BeautifulRobot&#8482;'s colour to the colour of an alliance.
     * 
     * @param color The colour to set to
     */
    public void setColor(Alliance color) {
        if(color == Alliance.Red) {
            writeCommand(Operation.COLOR, (byte) 0);
        }
        else if(color == Alliance.Blue) {
            writeCommand(Operation.COLOR, (byte) 1);
        }
        else {
            writeCommand(Operation.COLOR, (byte) 2);
        }
    }

    /**
     * Turns the BeautifulRobot&#8482; ON. By default, it is OFF.
     */
    public void turnOn() {
        writeCommand(Operation.ENABLE, (byte) 1);
    }
    /**
     * Turns the BeautifulRobot&#8482; OFF. By default, it is OFF.
     */
    public void turnOff() {
        writeCommand(Operation.ENABLE, (byte) 0);
    }
    /**
     * All the different patterns the BeautifulRobot&#8482; has.
     */
    public enum Pattern {
        SOLID((byte) 0), PULSATING((byte) 1), RAINBOW((byte) 2), MOVING_PULSE((byte) 3);

        private byte value;
        Pattern(byte value) {
            this.value = value;
        }
        public byte getValue() {
            return value;
        }
    }
    /**
     * Sets the pattern to be displayed by the BeautifulRobot&#8482;.
     * 
     * @param pattern The pattern to display
     */
    void setPattern(Pattern pattern) {
        writeCommand(Operation.MODE, pattern.getValue());
    }

    /**
     * Constructs a new BeautifulRobot&#8482;. This also calls {@link #init()}.
     */
    public BeautifulRobot() {
        super();
        init();
    }
}
