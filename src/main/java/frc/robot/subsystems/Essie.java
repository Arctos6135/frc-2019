/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Essie is our ball mechanism. :D
 */
public class Essie extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    /**
     * Reads Essie's photoelectric sensor to see if it has a cargo.
     * 
     * @return Whether there is cargo inside of Essie
     */
    public boolean hasCargo() {
        return RobotMap.essieVisisight.isBlocked();
    }
    /**
     * Sets Essie's motors to start intaking cargo.
     */
    public void startIntake() {
        RobotMap.essieMotorHigh.set(0);
        RobotMap.essieMotorLow.set(1.0);
    }
    /**
     * Sets Essie's motors to start outtaking through the lower exit (rocket ship level 1).
     */
    public void startOuttakeLow() {
        RobotMap.essieMotorHigh.set(-1.0);
        RobotMap.essieMotorLow.set(1.0);
    }
    /**
     * Sets Essie's motors to start outtaking through the upper exit (cargo ship).
     */
    public void startOuttakeHigh() {
        RobotMap.essieMotorHigh.set(1.0);
        RobotMap.essieMotorHigh.set(1.0);
    }
    /**
     * Stops Essie's motors.
     */
    public void stop() {
        RobotMap.essieMotorHigh.set(0);
        RobotMap.essieMotorLow.set(0);
    }

}