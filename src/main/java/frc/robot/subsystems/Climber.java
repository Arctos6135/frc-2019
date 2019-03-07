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
public class Climber extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    DoubleSolenoid.Value frontState;
    DoubleSolenoid.Value backState;

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

    public void setBackState(DoubleSolenoid.Value state) {
        this.backState = state;
        RobotMap.backClimber.set(state);
    }
    public DoubleSolenoid.Value getBackState() {
        return backState;
    }
    public void toggleBack() {
        if(backState == DoubleSolenoid.Value.kForward) {
            setBackState(DoubleSolenoid.Value.kReverse);
        }
        else {
            setBackState(DoubleSolenoid.Value.kForward);
        }
    }

    public Climber() {
        super();
        setFrontState(DoubleSolenoid.Value.kReverse);
        setBackState(DoubleSolenoid.Value.kReverse);
    }
    public Climber(String name) {
        super(name);
        setFrontState(DoubleSolenoid.Value.kReverse);
        setBackState(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
