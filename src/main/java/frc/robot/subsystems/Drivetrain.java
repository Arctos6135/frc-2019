/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;

public class Drivetrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new TeleopDrive());
    }

    // Having a speed multiplier allows for easy adjustment of top speeds
    private double speedMultiplier = 1.0;

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
    
    // Stores the previous left and right output values
    // Used by TeleopDrive for ramping
    double prevLeft = 0;
    double prevRight = 0;

    public double getPrevLeft() {
        return prevLeft;
    }
    public double getPrevRight() {
        return prevRight;
    }

    /**
     * Sets the left and right side motors of the drivetrain. 
     * The input values are first multiplied by the speed multiplier (see {@link #setSpeedMultiplier(double)}), 
     * and then constrained to [-1, 1].
     * @param left The left side motors percent output
     * @param right The right side motors percent output
     */
    public void setMotors(double left, double right) {
        prevLeft = left;
        prevRight = right;
        RobotMap.lVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, left * speedMultiplier)));
        // Invert right side
        RobotMap.rVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, -right * speedMultiplier)));
    }

    public enum Gear {
        LOW, HIGH;
    }
    /**
     * Sets the drivetrain gearboxes to be in either low or high gear.
     * 
     * Note that the low and high gear values may be switched depending on how the pneumatics are wired!
     * @param gear The new gear, {@link Gear#LOW} or {@link Gear#HIGH}.
     */
    public void setGear(Gear gear) {
        if(gear == Gear.HIGH) {
            RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
        }
        else {
            RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Returns the heading (yaw) of the robot.
     * @return The yaw of the robot
     */
    public double getHeading() {
        return RobotMap.ahrs.getFusedHeading();
    }
    /**
     * Resets the heading (yaw) of the robot.
     */
    public void resetHeading() {
        RobotMap.ahrs.reset();
    }

    public Drivetrain() {
        super();

        setMotors(0, 0);
        setGear(Gear.LOW);
        resetHeading();
    }
    public Drivetrain(String name) {
        super(name);

        setMotors(0, 0);
        setGear(Gear.LOW);
        resetHeading();
    }
}
