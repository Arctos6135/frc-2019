/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {

    public static final double DEADZONE = 0.15;

    static double rampBandLow = 0.07;
    static double rampBandHigh = 0.03;
    static boolean rampingOn = true;

    static boolean reverseDrive = false;
    static boolean precisionDrive = false;

    public TeleopDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
    }

    public static boolean isRamped() {
        return rampingOn;
    }
    public static void setRamping(boolean ramping) {
        rampingOn = ramping;
    }
    public static double getRampBandHigh() {
        return rampBandHigh;
    }
    public static double getRampBandLow() {
        return rampBandLow;
    }
    public static void setRampBandHigh(double band) {
        rampBandHigh = band;
    }
    public static void setRampBandLow(double band) {
        rampBandLow = band;
    }

    public static boolean isReversed() {
        return reverseDrive;
    }
    public static void setReversed(boolean reversed) {
        reverseDrive = reversed;
        if(reverseDrive) {
            Robot.mainCameraUrl.setString(Robot.REAR_CAMERA_URL);
            Robot.secondaryCameraUrl.setString(Robot.FRONT_CAMERA_URL);
        }
        else {
            Robot.mainCameraUrl.setString(Robot.FRONT_CAMERA_URL);
            Robot.secondaryCameraUrl.setString(Robot.REAR_CAMERA_URL);
        }
    }
    public static void reverse() {
        reverseDrive = !reverseDrive;
        if(reverseDrive) {
            Robot.mainCameraUrl.setString(Robot.REAR_CAMERA_URL);
            Robot.secondaryCameraUrl.setString(Robot.FRONT_CAMERA_URL);
        }
        else {
            Robot.mainCameraUrl.setString(Robot.FRONT_CAMERA_URL);
            Robot.secondaryCameraUrl.setString(Robot.REAR_CAMERA_URL);
        }
    }

    public static boolean isPrecisionDrive() {
        return precisionDrive;
    }
    public static void setPrecisionDrive(boolean precisionDrive) {
        TeleopDrive.precisionDrive = precisionDrive;
    }
    public static void togglePrecisionDrive() {
        precisionDrive = !precisionDrive;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drivetrain.enableSafety();
    }

    int logCounter = 0;
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        logCounter ++;
        // Handle regular driving
        double x = OI.driverController.getRawAxis(OI.Controls.DRIVE_LEFT_RIGHT);
        double y = -OI.driverController.getRawAxis(OI.Controls.DRIVE_FWD_REV);

        if(logCounter >= 20) {
            RobotLogger.logInfoFiner("Raw drive values: x=" + x + " y=" + y);
        }
        
        // See if the absolute value of X is greater than the deadzone
        if(Math.abs(x) > DEADZONE) {
            // Don't do anything with the value of X if it's valid
            // However, check and make sure the robot is in coast mode
            if(Robot.drivetrain.getNeutralMode() != NeutralMode.Coast) {
                Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
            }
        }
        // If it's not, then set it to 0 as it's probably just noise
        else {
            x = 0;
        }
        // Same logic as above
        if(Math.abs(y) > DEADZONE) {
            if(Robot.drivetrain.getNeutralMode() != NeutralMode.Coast) {
                Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
            }
        }
        else {
            y = 0;
        }
        
        // Square y and take x^4 and keep their signs
        // This is to give the driver greater control over the robot when the joystick is pushed less
        x = Math.copySign(Math.pow(x, 4), x);
        y = Math.copySign(y * y, y);

        if(Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH) {
            // Half the turning rate in high gear
            x /= 3;
        }

        if(reverseDrive) {
            y = -y;
        }

        double l = y + x, r = y - x;
        if(rampingOn) {
            double rampBand = Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH ? rampBandHigh : rampBandLow;
            l = Math.max(Robot.drivetrain.getPrevLeft() - rampBand, Math.min(Robot.drivetrain.getPrevLeft() + rampBand, l));
            r = Math.max(Robot.drivetrain.getPrevRight() - rampBand, Math.min(Robot.drivetrain.getPrevRight() + rampBand, r));
        }
        if(precisionDrive) {
            l /= 2;
            r /= 2;
        }

        if(logCounter >= 20) {
            RobotLogger.logInfoFiner("Drive output values: l=" + l + " r=" + r);
            logCounter = 0;
        }

        Robot.drivetrain.setMotors(l, r);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drivetrain.disableSafety();
        Robot.drivetrain.setMotors(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.drivetrain.disableSafety();
        Robot.drivetrain.setMotors(0, 0);
    }
}
