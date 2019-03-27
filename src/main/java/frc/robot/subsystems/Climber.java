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

    public enum State {
        EXTENDED, RETRACTED;

        public boolean value() {
            if(this == EXTENDED) {
                return true;
            }
            else {
                return false;
            }
        }
    }

    State frontState;
    State backState;

    public void setFrontState(State state) {
        this.frontState = state;
        RobotMap.frontClimber.set(state.value());
    }
    public State getFrontState() {
        return frontState;
    }
    public void toggleFront() {
        if(frontState == State.EXTENDED) {
            setFrontState(State.RETRACTED);
        }
        else {
            setFrontState(State.EXTENDED);
        }
    }

    public void setBackState(State state) {
        this.backState = state;
        RobotMap.backClimber.set(state.value());
    }
    public State getBackState() {
        return backState;
    }
    public void toggleBack() {
        if(backState == State.EXTENDED) {
            setBackState(State.RETRACTED);
        }
        else {
            setBackState(State.EXTENDED);
        }
    }

    public Climber() {
        super();
        setFrontState(State.RETRACTED);
        setBackState(State.RETRACTED);
    }
    public Climber(String name) {
        super(name);
        setFrontState(State.RETRACTED);
        setBackState(State.RETRACTED);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
