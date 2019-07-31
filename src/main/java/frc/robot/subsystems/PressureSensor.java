/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PressureSensor extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public PressureSensor() {

    }

    private double pressure;
    private final double supplyVoltage = 5;
    private final double requiredToClimb = 50;

    public void findPressure() {
        // apply a formula for turning voltage to psi value
        // 250 (voltageOut/supplyVoltage) - 25
        pressure = 250 * (RobotMap.pressureSensor.getAverageValue() / supplyVoltage) - 25;
    }
    public double getPressure() {
        findPressure();
        return pressure;
    }
    public boolean canClimb() {
        return getPressure() > requiredToClimb;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
