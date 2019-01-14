/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Drivetrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    private double speedMultiplier = 1.0;

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setMotors(double left, double right) {
        RobotMap.lVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, left * speedMultiplier)));
        // Invert right side
        RobotMap.rVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, -right * speedMultiplier)));
    }
}
