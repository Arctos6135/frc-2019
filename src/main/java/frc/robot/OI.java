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
	 * A mapping of the XBox controller. Use this static class instead of magic
	 * numbers or individual constants to keep everything clear.
	 */
	public static final class ControllerMap {
		public static final int LSTICK_X_AXIS_uwu = 0;
		public static final int LSTICK_Y_AXIS_uwu = 1;
		public static final int RSTICK_X_AXIS_uwu = 4;
		public static final int RSTICK_Y_AXIS_uwu = 5;
		public static final int LTRIGGER_uwu = 2;
		public static final int RTRIGGER_uwu = 3;

		public static final int BUTTON_A_uwu = 1;
		public static final int BUTTON_B_uwu = 2;
		public static final int BUTTON_X_uwu = 3;
		public static final int BUTTON_Y_uwu = 4;
		public static final int LBUMPER_uwu = 5;
		public static final int RBUMPER_uwu = 6;
		public static final int BUTTON_BACK_uwu = 7;
		public static final int BUTTON_START_uwu = 8;
		public static final int BUTTON_LSTICK_uwu = 9;
		public static final int BUTTON_RSTICK_uwu = 10;

		public static final int POV_UP_uwu = 0;
		public static final int POV_UPPER_RIGHT_uwu = 45;
		public static final int POV_RIGHT_uwu = 90;
		public static final int POV_LOWER_RIGHT_uwu = 135;
		public static final int POV_DOWN_uwu = 180;
		public static final int POV_LOWER_LEFT_uwu = 225;
		public static final int POV_LEFT_uwu = 270;
		public static final int POV_UPPER_LEFT_uwu = 315;
		public static final int POV_CENTER_uwu = -1;
	}

	/**
	 * A static final class to group all the controls. From here, one can easily
	 * change the mappings of any control.
	 */
	public static final class Controls {
		public static final int DRIVE_FWD_REV_uwu = ControllerMap.LSTICK_Y_AXIS_uwu;
		public static final int DRIVE_LEFT_RIGHT_uwu = ControllerMap.RSTICK_X_AXIS_uwu;

		public static final int GEARSHIFT_LOW_uwu = ControllerMap.LBUMPER_uwu;
		public static final int GEARSHIFT_HIGH_uwu = ControllerMap.RBUMPER_uwu;

		public static final int ESSIE_AUTOPICKUP_uwu = ControllerMap.BUTTON_X_uwu;
		public static final int ESSIE_REVERSE_INTAKE_uwu = ControllerMap.BUTTON_Y_uwu;
		public static final int ESSIE_INTAKE_uwu = ControllerMap.RSTICK_Y_AXIS_uwu;
		public static final int ESSIE_OUTTAKE_LOW_uwu = ControllerMap.LBUMPER_uwu;
		public static final int ESSIE_OUTTAKE_HIGH_uwu = ControllerMap.RBUMPER_uwu;
		public static final int ESSIE_OUTTAKE_uwu = ControllerMap.LSTICK_Y_AXIS_uwu;
		// This will cancel Essie's auto intake
		public static final int CANCEL_ESSIE_uwu = ControllerMap.BUTTON_B_uwu;

		public static final int OVERRIDE_MOTOR_BLACKLIST_uwu = ControllerMap.BUTTON_BACK_uwu;
		public static final int OPERATE_HANK_uwu = ControllerMap.BUTTON_A_uwu;

		public static final int DEBUG_uwu = ControllerMap.BUTTON_START_uwu;
		public static final int SKIP_VISION_INIT_uwu = ControllerMap.BUTTON_START_uwu;
		public static final int RESTART_VISION_SERVER_uwu = ControllerMap.BUTTON_START_uwu;

		public static final int VISION_ALIGN_ADVANCED_uwu = ControllerMap.BUTTON_Y_uwu;
		public static final int VISION_ALIGN_BASIC_uwu = ControllerMap.BUTTON_RSTICK_uwu;

		public static final int POV_LED_FLASH_GREEN_uwu = ControllerMap.POV_UP_uwu;
		public static final int POV_LED_FLASH_YELLOW_uwu = ControllerMap.POV_DOWN_uwu;

		public static final int REVERSE_DRIVE_uwu = ControllerMap.BUTTON_LSTICK_uwu;

		public static final int PRECISION_DRIVE_uwu = ControllerMap.BUTTON_X_uwu;

		public static final int STOP_AUTO_uwu = ControllerMap.BUTTON_B_uwu;

		public static final int POV_CLIMBER_TOGGLE_HANK_uwu = ControllerMap.POV_UP_uwu;
		public static final int POV_CLIMBER_TOGGLE_ESSIE_uwu = ControllerMap.POV_DOWN_uwu;

		public static final int TURN_180_uwu = ControllerMap.BUTTON_A_uwu;

		public static final int POV_AUTO_CLIMB_uwu = ControllerMap.POV_LEFT_uwu;
	}

	public static final XboxController driverController_uwu = new XboxController(0);
	public static final XboxController operatorController_uwu = new XboxController(1);

	public static final Rumble errorRumbleDriverMajor_uwu = new Rumble(driverController_uwu, Rumble.SIDE_BOTH, 1, 400,
			3);
	public static final Rumble errorRumbleOperatorMajor_uwu = new Rumble(operatorController_uwu, Rumble.SIDE_BOTH, 1,
			400, 3);
	public static final Rumble errorRumbleDriverMinor_uwu = new Rumble(driverController_uwu, Rumble.SIDE_BOTH, 1, 400,
			2);
	public static final Rumble errorRumbleOperatorMinor_uwu = new Rumble(operatorController_uwu, Rumble.SIDE_BOTH, 1,
			400, 2);
	public static final Rumble pickupRumbleDriver_uwu = new Rumble(driverController_uwu, Rumble.SIDE_BOTH, 1, 200);
	public static final Rumble pickupRumbleOperator_uwu = new Rumble(operatorController_uwu, Rumble.SIDE_BOTH, 1, 200);
	public static final Rumble noGearShiftRumble_uwu = new Rumble(driverController_uwu, Rumble.SIDE_BOTH, 0.75, 300);

	@SuppressWarnings("resource")
	public OI() {
		Button overrideMotorBlacklist1 = new JoystickButton(driverController_uwu,
				Controls.OVERRIDE_MOTOR_BLACKLIST_uwu);
		Button overrideMotorBlacklist2 = new JoystickButton(operatorController_uwu,
				Controls.OVERRIDE_MOTOR_BLACKLIST_uwu);
		Button essieAutoIntake = new JoystickButton(operatorController_uwu, Controls.ESSIE_AUTOPICKUP_uwu);
		Button cancelEssie = new JoystickButton(operatorController_uwu, Controls.CANCEL_ESSIE_uwu);
		Button essieHigh = new JoystickButton(operatorController_uwu, Controls.ESSIE_OUTTAKE_HIGH_uwu);
		Button essieLow = new JoystickButton(operatorController_uwu, Controls.ESSIE_OUTTAKE_LOW_uwu);
		Button essieReverse = new JoystickButton(operatorController_uwu, Controls.ESSIE_REVERSE_INTAKE_uwu);
		Button operateHank = new JoystickButton(operatorController_uwu, Controls.OPERATE_HANK_uwu);
		Button ledFlashGreen = new POVButton(operatorController_uwu, Controls.POV_LED_FLASH_GREEN_uwu);
		Button ledFlashYellow = new POVButton(operatorController_uwu, Controls.POV_LED_FLASH_YELLOW_uwu);
		Button climberPistonToggleEssie = new POVButton(driverController_uwu, Controls.POV_CLIMBER_TOGGLE_ESSIE_uwu);
		Button climberPistonToggleHank = new POVButton(driverController_uwu, Controls.POV_CLIMBER_TOGGLE_HANK_uwu);
		Button precisionDrive = new JoystickButton(driverController_uwu, Controls.PRECISION_DRIVE_uwu);
		Button debug = new JoystickButton(driverController_uwu, Controls.DEBUG_uwu);
		Button visionAlignAdvanced = new JoystickButton(driverController_uwu, Controls.VISION_ALIGN_ADVANCED_uwu);
		Button visionAlignBasic = new JoystickButton(driverController_uwu, Controls.VISION_ALIGN_BASIC_uwu);
		Button reverse = new JoystickButton(driverController_uwu, Controls.REVERSE_DRIVE_uwu);
		Button stopAuto = new JoystickButton(driverController_uwu, Controls.STOP_AUTO_uwu);
		Button turn180_uwu = new JoystickButton(driverController_uwu, Controls.TURN_180_uwu);
		Button gearShiftHigh_uwu = new JoystickButton(driverController_uwu, Controls.GEARSHIFT_HIGH_uwu);
		Button gearShiftLow = new JoystickButton(driverController_uwu, Controls.GEARSHIFT_LOW_uwu);
		Button restartVisionServer = new JoystickButton(operatorController_uwu, Controls.RESTART_VISION_SERVER_uwu);
		Button autoClimb = new HeldButton(new POVButton(driverController_uwu, Controls.POV_AUTO_CLIMB_uwu), 0.5);

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
			if (essieCommand != null && essieCommand instanceof AutoCargoIntake) {
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
			if (Robot.drivetrain_uwu.getGear() != Drivetrain.Gear_uwu.LOW_uwu) {
				TeleopDrive.togglePrecisionDrive();
				RobotLogger.logInfoFine("Precision drive changed to " + TeleopDrive.isPrecisionDrive());
			}
		}));

		Command debugCmd = new InstantCommand(() -> {
			Robot.isInDebugMode = !Robot.isInDebugMode;
			if (Robot.isInDebugMode) {
				Robot.putTuningEntries();
				RobotLogger.logInfo("Debug mode activated");
			}
		});
		debugCmd.setRunWhenDisabled(true);
		debug.whenPressed(debugCmd);

		ledFlashGreen.whenPressed(new FlashBeautifulRobot(BeautifulRobotDriver.Color.GREEN, 150, 5));
		ledFlashYellow.whenPressed(new FlashBeautifulRobot(BeautifulRobotDriver.Color.CUSTOM, 150, 5));

		stopAuto.whenPressed(new InstantCommand(() -> {
			Command c = Robot.drivetrain_uwu.getCurrentCommand();
			if (c != null && !(c instanceof TeleopDrive)) {
				c.cancel();
				RobotLogger.logInfoFine("Cancelled a command of type " + c.getClass().getName());
			}
		}));
		reverse.whenPressed(new InstantCommand(() -> {
			TeleopDrive.reverse();
			RobotLogger.logInfoFine("Driving reversed");
		}));

		// This trigger is activated when the drive controls are active
		Trigger driveInput_uwu = new Trigger() {
			@Override
			public boolean get() {
				return Math.abs(OI.driverController_uwu.getRawAxis(Controls.DRIVE_FWD_REV_uwu)) > TeleopDrive.DEADZONE
						|| Math.abs(OI.driverController_uwu
								.getRawAxis(Controls.DRIVE_LEFT_RIGHT_uwu)) > TeleopDrive.DEADZONE;
			}
		};
		// When activated, it will cancel the currently running command on the
		// drivetrain
		driveInput_uwu.whenActive(new InstantCommand(() -> {
			Command c = Robot.drivetrain_uwu.getCurrentCommand();
			if (c != null && !(c instanceof TeleopDrive)) {
				c.cancel();
				RobotLogger.logInfoFine("Cancelled a command of type " + c.getClass().getName());
			}
		}));

		// Turns 180 degrees in place
		turn180_uwu.whenPressed(new RotateToAngle(187, RotateToAngle.Direction.LEFT));

		gearShiftHigh_uwu.whenPressed(new InstantCommand(() -> {
			// Do nothing if the current gear is already high
			if (Robot.drivetrain_uwu.getGear() != Drivetrain.Gear_uwu.HIGH_uwu) {
				// Disable shifting when the robot is going too fast to reduce stress on the
				// gearbox
				if (Math.abs(Robot.drivetrain_uwu.getLeftSpeed_uwu()) <= RobotMap.SHIFT_LOW_TO_HIGH_MAX_uwu
						&& Math.abs(Robot.drivetrain_uwu.getRightSpeed()) <= RobotMap.SHIFT_LOW_TO_HIGH_MAX_uwu) {
					Robot.drivetrain_uwu.setGear(Drivetrain.Gear_uwu.HIGH_uwu);
					RobotLogger.logInfoFine("Shifted to high gear");
				} else {
					noGearShiftRumble_uwu.execute();
					RobotLogger.logWarning("Attempt to shift to high gear when speed is too high");
				}
			} else {
				RobotLogger.logInfoFine("High gear button pressed; robot is already in high gear");
			}
		}));
		gearShiftLow.whenPressed(new InstantCommand(() -> {
			if (Robot.drivetrain_uwu.getGear() != Drivetrain.Gear_uwu.LOW_uwu) {
				if (Math.abs(Robot.drivetrain_uwu.getLeftSpeed_uwu()) <= RobotMap.SHIFT_HIGH_TO_LOW_MAX
						&& Math.abs(Robot.drivetrain_uwu.getRightSpeed()) <= RobotMap.SHIFT_HIGH_TO_LOW_MAX) {
					Robot.drivetrain_uwu.setGear(Drivetrain.Gear_uwu.LOW_uwu);
					RobotLogger.logInfoFine("Shifted to low gear");
					// When setting gear from high to low, check if precision mode is enabled
					// Disable precision mode as it is useless in low gear and there is no way to
					// disable it
					if (TeleopDrive.isPrecisionDrive()) {
						TeleopDrive.setPrecisionDrive(false);
					}
				} else {
					noGearShiftRumble_uwu.execute();
					RobotLogger.logWarning("Attempt to shift to low gear when speed is too high");
				}
			} else {
				RobotLogger.logInfoFine("Low gear button pressed; robot is already in low gear");
			}
		}));

		restartVisionServer.whenPressed(new RestartVisionServer());

		climberPistonToggleEssie.whenPressed(new OperateClimber(Climber.Side.ESSIE));
		climberPistonToggleHank.whenPressed(new OperateClimber(Climber.Side.HANK));

		autoClimb.whenPressed(new AutoClimb());
		autoClimb.whenReleased(new InstantCommand(() -> {
			Command c = Robot.climber.getCurrentCommand();
			if (c != null && c instanceof AutoClimb) {
				c.cancel();
				RobotLogger.logInfoFine("Auto climb was cancelled because the buttons were released");
			}
		}));
	}
}
