/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {

    private static final double DEADZONE = 0.15;

    static double rampBand = 0.07;
    static boolean rampingOn = true;

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

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // Handle regular driving
        double x = OI.driverController.getRawAxis(OI.Controls.DRIVE_LEFT_RIGHT);
        double y = OI.driverController.getRawAxis(OI.Controls.DRIVE_FWD_REV);

        x = Math.abs(x) > DEADZONE ? x : 0;
        y = Math.abs(y) > DEADZONE ? y : 0;

        x = Math.copySign(x * x, x);
        y = Math.copySign(y * y, y);

        double l = y + x, r = y - x;
        if(rampingOn) {
            l = Math.max(Robot.drivetrain.getPrevLeft() - rampBand, Math.min(Robot.drivetrain.getPrevLeft() + rampBand, l));
            r = Math.max(Robot.drivetrain.getPrevRight() - rampBand, Math.min(Robot.drivetrain.getPrevRight() + rampBand, r));
        }

        Robot.drivetrain.setMotors(l, r);

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