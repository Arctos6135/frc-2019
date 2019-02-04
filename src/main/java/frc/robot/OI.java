/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.FlashBeautifulRobot;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());

    /**
     * A mapping of the XBox controller. Use this static class instead of 
     * magic numbers or individual constants to keep everything clear.
     */
    public static final class ControllerMap {
        public static final int LSTICK_X_AXIS = 0;
        public static final int LSTICK_Y_AXIS = 1;
        public static final int RSTICK_X_AXIS = 4;
        public static final int RSTICK_Y_AXIS = 5;
        public static final int LTRIGGER = 2;
        public static final int RTRIGGER = 3;

        public static final int BUTTON_A = 1;
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int LBUMPER = 5;
        public static final int RBUMPER = 6;
        public static final int BUTTON_BACK = 7;
        public static final int BUTTON_START = 8;
        public static final int BUTTON_LSTICK = 9;
        public static final int BUTTON_RSTICK = 10;

        public static final int POV_UP = 0;
        public static final int POV_UPPER_RIGHT = 45;
        public static final int POV_RIGHT = 90;
        public static final int POV_LOWER_RIGHT = 135;
        public static final int POV_DOWN = 180;
        public static final int POV_LOWER_LEFT = 225;
        public static final int POV_LEFT = 270;
        public static final int POV_UPPER_LEFT = 315;
        public static final int POV_CENTER = -1;
    }
    /**
     * A static final class to group all the controls. From here, one can easily change the mappings of any control.
     */
    public static final class Controls {
        public static final int DRIVE_FWD_REV = ControllerMap.LSTICK_Y_AXIS;
        public static final int DRIVE_LEFT_RIGHT = ControllerMap.RSTICK_X_AXIS;

        public static final int GEARSHIFT_LOW = ControllerMap.LBUMPER;
        public static final int GEARSHIFT_HIGH = ControllerMap.RBUMPER;

        public static final int DEBUG = ControllerMap.BUTTON_START;

        public static final int POV_LED_FLASH_SHORT = ControllerMap.POV_UP;
        public static final int POV_LED_FLASH_LONG = ControllerMap.POV_DOWN;
    }

    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);

    @SuppressWarnings("resource")
    public OI() {
        JoystickButton debug = new JoystickButton(driverController, Controls.DEBUG);
        POVButton ledFlashShort = new POVButton(operatorController, Controls.POV_LED_FLASH_SHORT);
        POVButton ledFlashLong = new POVButton(operatorController, Controls.POV_LED_FLASH_LONG);

        Command debugCmd = new InstantCommand() {
            @Override
            protected void initialize() {
                Robot.isInDebugMode = !Robot.isInDebugMode;
            }
        };
        debugCmd.setRunWhenDisabled(true);
        debug.whenPressed(debugCmd);

        ledFlashShort.whenPressed(new FlashBeautifulRobot(3.5, 1));
        ledFlashLong.whenPressed(new FlashBeautifulRobot(1.5, 13));
    }
}
