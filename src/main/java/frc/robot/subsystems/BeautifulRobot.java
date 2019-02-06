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
        // Set mode to solid colour
        writeCommand(Operation.MODE, (byte) 0);
    }
    /**
     * Sets the BeautifulRobot&#8482;'s colour to the colour of an alliance.
     * 
     * @param color The alliance colour to set to
     */
    public void setAlliance(Alliance color) {
        if(color == Alliance.Red) {
            setColor(Color.RED);
        }
        else if(color == Alliance.Blue) {
            setColor(Color.BLUE);
        }
        else {
            setColor(Color.GREEN);
        }
    }

    /**
     * A colour of the BeautifulRobot&#8482;.
     */
    public enum Color {
        RED((byte) 0), BLUE((byte) 1), GREEN((byte) 2), YELLOW((byte) 3);

        byte code;
        Color(byte code) {
            this.code = code;
        }
        public byte getCode() {
            return code;
        }
    }
    private Color color = Color.GREEN;

    /**
     * Sets the colour of the BeautifulRobot&#8482;.
     * 
     * @param color The colour to set to
     */
    public void setColor(Color color) {
        this.color = color;
        writeCommand(Operation.COLOR, color.getCode());
    }
    /**
     * Gets the colour of the BeautifulRobot&#8482;, last set by the {@link #setColor(Color)} method.
     * @return The color
     */
    public Color getColor() {
        return color;
    }

    private boolean on = false;
    /**
     * Turns the BeautifulRobot&#8482; ON. By default, it is OFF.
     */
    public void turnOn() {
        on = true;
        writeCommand(Operation.ENABLE, (byte) 1);
    }
    /**
     * Turns the BeautifulRobot&#8482; OFF. By default, it is OFF.
     */
    public void turnOff() {
        on = false;
        writeCommand(Operation.ENABLE, (byte) 0);
    }
    /**
     * Returns whether the BeautifulRobot&#8482; is on. By default, it is OFF.
     * @return Whether the BeautifulRobot&#8482; is on
     */
    public boolean isOn() {
        return on;
    }
    /**
     * All the different patterns the BeautifulRobot&#8482; has.
     */
    public enum Pattern {
        SOLID((byte) 0), PULSATING((byte) 1), RAINBOW((byte) 2), MOVING_PULSE((byte) 3), PROGRESS_BAR((byte) 4);

        private byte value;
        Pattern(byte value) {
            this.value = value;
        }
        public byte getValue() {
            return value;
        }
    }

    private Pattern pattern = Pattern.SOLID;
    /**
     * Sets the pattern to be displayed by the BeautifulRobot&#8482;.
     * 
     * @param pattern The pattern to display
     */
    public void setPattern(Pattern pattern) {
        this.pattern = pattern;
        writeCommand(Operation.MODE, pattern.getValue());
    }

    /**
     * Gets the pattern to be displayed by the BeautifulRobot&#8482;.
     * 
     * @return The pattern
     */
    public Pattern getPattern() {
        return pattern;
    }

    /**
     * Constructs a new BeautifulRobot&#8482;. This also calls {@link #init()}.
     */
    public BeautifulRobot() {
        super();
        init();
    }
    /**
     * Constructs a new BeautifulRobot&#8482;. This also calls {@link #init()}.
     * 
     * @param name The name of the subsystem
     */
    public BeautifulRobot(String name) {
        super(name);
        init();
    }
}
