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

        public DoubleSolenoid.Value value() {
            if(this == EXTENDED) {
                return DoubleSolenoid.Value.kForward;
            }
            else {
                return DoubleSolenoid.Value.kReverse;
            }
        }
    }

    public void setFrontState(State state) {
        RobotMap.frontClimber.set(state.value());
    }
    public State getFrontState() {
        // If get() returns false it means that the piston is extended
        return !RobotMap.frontLeftMRS.get() && !RobotMap.frontRightMRS.get() ? State.EXTENDED : State.RETRACTED;
    }
    public void toggleFront() {
        if(getFrontState() == State.EXTENDED) {
            setFrontState(State.RETRACTED);
        }
        else {
            setFrontState(State.EXTENDED);
        }
    }

    public void setBackState(State state) {
        RobotMap.backClimber.set(state.value());
    }
    public State getBackState() {
        return !RobotMap.backleftMRS.get() && !RobotMap.backRightMRS.get() ? State.EXTENDED : State.RETRACTED;
    }
    public void toggleBack() {
        if(getBackState() == State.EXTENDED) {
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
