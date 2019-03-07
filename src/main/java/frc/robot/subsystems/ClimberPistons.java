/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
  * Add your docs here.
  */
public class ClimberPistons extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    DoubleSolenoid.Value frontState;

    public void setFrontState(DoubleSolenoid.Value state) {
        this.frontState = state;
        RobotMap.frontClimber.set(state);
    }
    public DoubleSolenoid.Value getFrontState() {
        return frontState;
    }
    public void toggleFront() {
        if(frontState == DoubleSolenoid.Value.kForward) {
            setFrontState(DoubleSolenoid.Value.kReverse);
        }
        else {
            setFrontState(DoubleSolenoid.Value.kForward);
        }
    }

    public ClimberPistons() {
        super();
        setFrontState(DoubleSolenoid.Value.kReverse);
    }
    public ClimberPistons(String name) {
        super(name);
        setFrontState(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
