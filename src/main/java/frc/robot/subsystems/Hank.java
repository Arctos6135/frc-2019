/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hank extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public Hank() {
        super();
        retract();
    }

    public void pushOut() {
        RobotMap.hankSolenoidA.set(DoubleSolenoid.Value.kForward);
        RobotMap.hankSolenoidB.set(DoubleSolenoid.Value.kForward);
    }
    public void retract() {
        RobotMap.hankSolenoidA.set(DoubleSolenoid.Value.kReverse);
        RobotMap.hankSolenoidB.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
