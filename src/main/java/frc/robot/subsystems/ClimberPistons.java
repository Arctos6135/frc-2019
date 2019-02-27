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

    DoubleSolenoid.Value state;

    public void setState(DoubleSolenoid.Value state) {
        this.state = state;
        RobotMap.climberSolenoid.set(state);
    }
    public DoubleSolenoid.Value getState() {
        return state;
    }
    public void toggle() {
        if(state == DoubleSolenoid.Value.kForward) {
            setState(DoubleSolenoid.Value.kReverse);
        }
        else {
            setState(DoubleSolenoid.Value.kForward);
        }
    }

    public ClimberPistons() {
        super();
        setState(DoubleSolenoid.Value.kReverse);
    }
    public ClimberPistons(String name) {
        super(name);
        setState(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
