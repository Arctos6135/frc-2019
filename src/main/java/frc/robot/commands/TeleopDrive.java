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
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {

    public static final double DEADZONE = 0.15;

    static double rampBand = 0.07;
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
    public static double getRampBand() {
        return rampBand;
    }
    public static void setRampBand(double band) {
        rampBand = band;
    }

    public static boolean isReversed() {
        return reverseDrive;
    }
    public static void setReversed(boolean reversed) {
        reverseDrive = reversed;
    }
    public static void reverse() {
        reverseDrive = !reverseDrive;
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
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // Handle regular driving
        double x = OI.driverController.getRawAxis(OI.Controls.DRIVE_LEFT_RIGHT);
        double y = -OI.driverController.getRawAxis(OI.Controls.DRIVE_FWD_REV);
        
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

        if(reverseDrive) {
            y = -y;
        }

        double l = y + x, r = y - x;
        if(rampingOn) {
            l = Math.max(Robot.drivetrain.getPrevLeft() - rampBand, Math.min(Robot.drivetrain.getPrevLeft() + rampBand, l));
            r = Math.max(Robot.drivetrain.getPrevRight() - rampBand, Math.min(Robot.drivetrain.getPrevRight() + rampBand, r));
        }

        if(precisionDrive) {
            Robot.drivetrain.setMotors(l / 2, r / 2);
        }
        else {
            Robot.drivetrain.setMotors(l, r);
        }

        // Handle gear shifts
        boolean shiftLow = OI.driverController.getRawButton(OI.Controls.GEARSHIFT_LOW);
        boolean shiftHigh = OI.driverController.getRawButton(OI.Controls.GEARSHIFT_HIGH);
        // Both high and low are pressed
        // Maybe someone is mashing the controller.
        // Or maybe the driver is just really stupid.
        // Whatever it is, it doesn't concern us.
        if(shiftLow && shiftHigh) {
            return;
        }
        // Only one should be down at a time
        else if(shiftLow) {
            Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        }
        else if(shiftHigh) {
            Robot.drivetrain.setGear(Drivetrain.Gear.HIGH);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drivetrain.setMotors(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.drivetrain.setMotors(0, 0);
    }
}
