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

public class Claw extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public enum State {
        OPEN, CLOSED;
    }

    private State state;
    public void set(State state) {
        setState(state);
    }
    public void setState(State state) {
        this.state = state;
        if(state == State.OPEN) {
            RobotMap.clawSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        else {
            RobotMap.clawSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }
    public State getState() {
        return state;
    }

    public Claw() {
        super();
        setState(State.OPEN);
    }
    public Claw(String name) {
        super(name);
        setState(State.OPEN);
    }
}
