/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.AdvancedVisionAlign;
import frc.robot.commands.AutoCargoIntake;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.FlashBeautifulRobot;
import frc.robot.commands.OperateClimber;
import frc.robot.commands.OperateEssie;
import frc.robot.commands.OperateHank;
import frc.robot.commands.RestartVisionServer;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.ShutdownJetson;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.VisionAlign;
import frc.robot.misc.BeautifulRobotDriver;
import frc.robot.misc.RobotLogger;
import frc.robot.misc.Rumble;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.triggers.HeldButton;

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

        public static final int ESSIE_AUTOPICKUP = ControllerMap.BUTTON_X;
        public static final int ESSIE_REVERSE_INTAKE = ControllerMap.BUTTON_Y;
        public static final int ESSIE_INTAKE = ControllerMap.RSTICK_Y_AXIS;
        public static final int ESSIE_OUTTAKE_LOW = ControllerMap.LBUMPER;
        public static final int ESSIE_OUTTAKE_HIGH = ControllerMap.RBUMPER;
        public static final int ESSIE_OUTTAKE = ControllerMap.LSTICK_Y_AXIS;
        // This will cancel Essie's auto intake
        public static final int CANCEL_ESSIE = ControllerMap.BUTTON_B;

        public static final int OVERRIDE_MOTOR_BLACKLIST = ControllerMap.BUTTON_BACK;
        public static final int OPERATE_HANK = ControllerMap.BUTTON_A;
        
        public static final int DEBUG = ControllerMap.BUTTON_START;
        public static final int SKIP_VISION_INIT = ControllerMap.BUTTON_START;
        public static final int RESTART_VISION_SERVER = ControllerMap.BUTTON_START;
        
        public static final int VISION_ALIGN_ADVANCED = ControllerMap.BUTTON_Y;
        public static final int VISION_ALIGN_BASIC = ControllerMap.BUTTON_RSTICK;

        public static final int POV_LED_FLASH_GREEN = ControllerMap.POV_UP;
        public static final int POV_LED_FLASH_YELLOW = ControllerMap.POV_DOWN;
        
        public static final int REVERSE_DRIVE = ControllerMap.BUTTON_LSTICK;

        public static final int PRECISION_DRIVE = ControllerMap.BUTTON_X;

        public static final int STOP_AUTO = ControllerMap.BUTTON_B;

        public static final int POV_CLIMBER_TOGGLE_HANK = ControllerMap.POV_UP;
        public static final int POV_CLIMBER_TOGGLE_ESSIE = ControllerMap.POV_DOWN;

        public static final int TURN_180 = ControllerMap.BUTTON_A;

        public static final int POV_AUTO_CLIMB = ControllerMap.POV_LEFT;
    }

    public static final XboxController driverController = new XboxController(0);
    public static final XboxController operatorController = new XboxController(1);

    public static final Rumble errorRumbleDriverMajor = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 400, 3);
    public static final Rumble errorRumbleOperatorMajor = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 400, 3);
    public static final Rumble errorRumbleDriverMinor = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 400, 2);
    public static final Rumble errorRumbleOperatorMinor = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 400, 2);
    public static final Rumble pickupRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 200);
    public static final Rumble pickupRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 200);
    public static final Rumble noGearShiftRumble = new Rumble(driverController, Rumble.SIDE_BOTH, 0.75, 300);
    
    @SuppressWarnings("resource")
    public OI() {
        Button overrideMotorBlacklist1 = new JoystickButton(driverController, Controls.OVERRIDE_MOTOR_BLACKLIST);
        Button overrideMotorBlacklist2 = new JoystickButton(operatorController, Controls.OVERRIDE_MOTOR_BLACKLIST);
        Button essieAutoIntake = new JoystickButton(operatorController, Controls.ESSIE_AUTOPICKUP);
        Button cancelEssie = new JoystickButton(operatorController, Controls.CANCEL_ESSIE);
        Button essieHigh = new JoystickButton(operatorController, Controls.ESSIE_OUTTAKE_HIGH);
        Button essieLow = new JoystickButton(operatorController, Controls.ESSIE_OUTTAKE_LOW);
        Button essieReverse = new JoystickButton(operatorController, Controls.ESSIE_REVERSE_INTAKE);
        Button operateHank = new JoystickButton(operatorController, Controls.OPERATE_HANK);
        Button ledFlashGreen = new POVButton(operatorController, Controls.POV_LED_FLASH_GREEN);
        Button ledFlashYellow = new POVButton(operatorController, Controls.POV_LED_FLASH_YELLOW);
        Button climberPistonToggleEssie = new POVButton(driverController, Controls.POV_CLIMBER_TOGGLE_ESSIE);
        Button climberPistonToggleHank = new POVButton(driverController, Controls.POV_CLIMBER_TOGGLE_HANK);
        Button precisionDrive = new JoystickButton(driverController, Controls.PRECISION_DRIVE);
        Button debug = new JoystickButton(driverController, Controls.DEBUG);
        Button visionAlignAdvanced = new JoystickButton(driverController, Controls.VISION_ALIGN_ADVANCED);
        Button visionAlignBasic = new JoystickButton(driverController, Controls.VISION_ALIGN_BASIC);
        Button reverse = new JoystickButton(driverController, Controls.REVERSE_DRIVE);
        Button stopAuto = new JoystickButton(driverController, Controls.STOP_AUTO);
        Button turn180 = new JoystickButton(driverController, Controls.TURN_180);
        Button gearShiftHigh = new JoystickButton(driverController, Controls.GEARSHIFT_HIGH);
        Button gearShiftLow = new JoystickButton(driverController, Controls.GEARSHIFT_LOW);
        Button restartVisionServer = new JoystickButton(operatorController, Controls.RESTART_VISION_SERVER);
        Button autoClimb = new HeldButton(new POVButton(driverController, Controls.POV_AUTO_CLIMB), 0.5);

        overrideMotorBlacklist1.whenActive(new InstantCommand(() -> {
            RobotMap.essieMotorHigh.overrideBlacklist();
            RobotMap.essieMotorLow.overrideBlacklist();
            RobotLogger.logWarning("Motor protection manually overridden");
        }));
        overrideMotorBlacklist2.whenActive(new InstantCommand(() -> {
            RobotMap.essieMotorHigh.overrideBlacklist();
            RobotMap.essieMotorLow.overrideBlacklist();
            RobotLogger.logWarning("Motor protection manually overridden");
        }));

        essieAutoIntake.whenPressed(new AutoCargoIntake());
        essieHigh.whileHeld(new OperateEssie(OperateEssie.Mode.OUT_HIGH));
        essieLow.whileHeld(new OperateEssie(OperateEssie.Mode.OUT_LOW));
        essieReverse.whileHeld(new OperateEssie(OperateEssie.Mode.REVERSE));

        cancelEssie.whenActive(new InstantCommand(() -> {
            Command essieCommand = Robot.essie.getCurrentCommand();
            if(essieCommand != null && essieCommand instanceof AutoCargoIntake) {
                essieCommand.cancel();
                RobotLogger.logInfoFine("Essie autopickup cancelled");
            }
        }));

        operateHank.whileHeld(new OperateHank());

        // User button on the rio shuts down the Jetson
        Trigger shutdownJetson = new Trigger() {
            @Override
            public boolean get() {
                return RobotController.getUserButton();
            }
        };
        shutdownJetson.whileActive(new ShutdownJetson());

        visionAlignAdvanced.whenPressed(new AdvancedVisionAlign());
        visionAlignBasic.whenPressed(new VisionAlign());
        precisionDrive.whenPressed(new InstantCommand(() -> {
            // Precision drive is disabled when the robot is in low gear,
            // as the robot already goes very slowly anyways.
            if(Robot.drivetrain.getGear() != Drivetrain.Gear.LOW) {
                TeleopDrive.togglePrecisionDrive();
                RobotLogger.logInfoFine("Precision drive changed to " + TeleopDrive.isPrecisionDrive());
            }
        }));

        Command debugCmd = new InstantCommand(() -> {
            Robot.isInDebugMode = !Robot.isInDebugMode;
            if(Robot.isInDebugMode) {
                Robot.putTuningEntries();
                RobotLogger.logInfo("Debug mode activated");
            }
        });
        debugCmd.setRunWhenDisabled(true);
        debug.whenPressed(debugCmd);
        
        ledFlashGreen.whenPressed(new FlashBeautifulRobot(BeautifulRobotDriver.Color.GREEN, 150, 5));
        ledFlashYellow.whenPressed(new FlashBeautifulRobot(BeautifulRobotDriver.Color.CUSTOM, 150, 5));

        stopAuto.whenPressed(new InstantCommand(() -> {
            Command c = Robot.drivetrain.getCurrentCommand();
            if(c != null && !(c instanceof TeleopDrive)) {
                c.cancel();
                RobotLogger.logInfoFine("Cancelled a command of type " + c.getClass().getName());
            }
        }));
        reverse.whenPressed(new InstantCommand(() -> {
            TeleopDrive.reverse();
            RobotLogger.logInfoFine("Driving reversed");
        }));

        // This trigger is activated when the drive controls are active
        Trigger driveInput = new Trigger() {
            @Override
            public boolean get() {
                return Math.abs(OI.driverController.getRawAxis(Controls.DRIVE_FWD_REV)) > TeleopDrive.DEADZONE
                        || Math.abs(OI.driverController.getRawAxis(Controls.DRIVE_LEFT_RIGHT)) > TeleopDrive.DEADZONE;
            }
        };
        // When activated, it will cancel the currently running command on the drivetrain
        driveInput.whenActive(new InstantCommand(() -> {
            Command c = Robot.drivetrain.getCurrentCommand();
            if(c != null && !(c instanceof TeleopDrive)) {
                c.cancel();
                RobotLogger.logInfoFine("Cancelled a command of type " + c.getClass().getName());
            }
        }));

        // Turns 180 degrees in place
        turn180.whenPressed(new RotateToAngle(187, RotateToAngle.Direction.LEFT));

        gearShiftHigh.whenPressed(new InstantCommand(() -> {
            // Do nothing if the current gear is already high
            if(Robot.drivetrain.getGear() != Drivetrain.Gear.HIGH) {
                // Disable shifting when the robot is going too fast to reduce stress on the gearbox
                if(Math.abs(Robot.drivetrain.getLeftSpeed()) <= RobotMap.SHIFT_LOW_TO_HIGH_MAX
                        && Math.abs(Robot.drivetrain.getRightSpeed()) <= RobotMap.SHIFT_LOW_TO_HIGH_MAX) {
                    Robot.drivetrain.setGear(Drivetrain.Gear.HIGH);
                    RobotLogger.logInfoFine("Shifted to high gear");
                }
                else {
                    noGearShiftRumble.execute();
                    RobotLogger.logWarning("Attempt to shift to high gear when speed is too high");
                }
            }
            else {
                RobotLogger.logInfoFine("High gear button pressed; robot is already in high gear");
            }
        }));
        gearShiftLow.whenPressed(new InstantCommand(() -> {
            if(Robot.drivetrain.getGear() != Drivetrain.Gear.LOW) {
                if(Math.abs(Robot.drivetrain.getLeftSpeed()) <= RobotMap.SHIFT_HIGH_TO_LOW_MAX
                        && Math.abs(Robot.drivetrain.getRightSpeed()) <= RobotMap.SHIFT_HIGH_TO_LOW_MAX) {
                    Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
                    RobotLogger.logInfoFine("Shifted to low gear");
                    // When setting gear from high to low, check if precision mode is enabled
                    // Disable precision mode as it is useless in low gear and there is no way to disable it
                    if(TeleopDrive.isPrecisionDrive()) {
                        TeleopDrive.setPrecisionDrive(false);
                    }
                }
                else {
                    noGearShiftRumble.execute();
                    RobotLogger.logWarning("Attempt to shift to low gear when speed is too high");
                }
            }
            else {
                RobotLogger.logInfoFine("Low gear button pressed; robot is already in low gear");
            }
        }));

        restartVisionServer.whenPressed(new RestartVisionServer());

        climberPistonToggleEssie.whenPressed(new OperateClimber(Climber.Side.ESSIE));
        climberPistonToggleHank.whenPressed(new OperateClimber(Climber.Side.HANK));

        autoClimb.whenPressed(new AutoClimb());
        autoClimb.whenReleased(new InstantCommand(() -> {
            Command c = Robot.climber.getCurrentCommand();
            if(c != null && c instanceof AutoClimb) {
                c.cancel();
                RobotLogger.logInfoFine("Auto climb was cancelled because the buttons were released");
            }
        }));
    }
}
