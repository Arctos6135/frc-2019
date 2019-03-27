/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.misc.RobotLogger;

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
        public State opposite() {
            if(this == EXTENDED) {
                return RETRACTED;
            }
            else {
                return EXTENDED;
            }
        }
    }

    public enum Side {
        FRONT, BACK;
    }

    public void setFrontState(State state) {
        setFrontState(state, false);
    }
    public void setFrontState(State state, boolean wait) {
        RobotLogger.logInfoFine("Setting front climber pistons to state " + state.toString());
        RobotMap.frontClimber.set(state.value());

        if(wait) {
            if(state == State.RETRACTED) {
                try {
                    Thread.sleep(500);
                }
                catch(InterruptedException e) {
                    RobotLogger.logError("Unexpected InterruptedException in Thread.sleep() in Climber.setFrontState()");
                    return;
                }
            }
            else {
                double start = Timer.getFPGATimestamp();
                while(getFrontState() != State.EXTENDED) {
                    try {
                        Thread.sleep(50);
                    }
                    catch(InterruptedException e) {
                        RobotLogger.logError("Unexpected InterruptedException in Thread.sleep() in Climber.setFrontState()");
                        return;
                    }
                    
                    if(Timer.getFPGATimestamp() - start >= 2.0) {
                        RobotLogger.logError("Waiting for front pistons to extend timed out (2 seconds)");
                        OI.errorRumbleDriverMajor.execute();
                        OI.errorRumbleOperatorMajor.execute();
                        return;
                    }
                }
            }
        }
    }
    public State getFrontState() {
        // If get() returns false it means that the piston is extended
        return !RobotMap.frontMRS.get() ? State.EXTENDED : State.RETRACTED;
    }
    public void toggleFront() {
        toggleFront(false);
    }
    public void toggleFront(boolean wait) {
        setFrontState(getFrontState().opposite(), wait);
    }

    public void setBackState(State state) {
        setBackState(state, false);
    }
    public void setBackState(State state, boolean wait) {
        RobotLogger.logInfoFine("Setting back climber pistons to state " + state.toString());
        RobotMap.backClimber.set(state.value());

        if(wait) {
            if(state == State.RETRACTED) {
                try {
                    Thread.sleep(500);
                }
                catch(InterruptedException e) {
                    RobotLogger.logError("Unexpected InterruptedException in Thread.sleep() in Climber.setBackState()");
                    return;
                }
            }
            else {
                double start = Timer.getFPGATimestamp();
                while(getBackState() != State.EXTENDED) {
                    try {
                        Thread.sleep(50);
                    }
                    catch(InterruptedException e) {
                        RobotLogger.logError("Unexpected InterruptedException in Thread.sleep() in Climber.setBackState()");
                        return;
                    }
                    
                    if(Timer.getFPGATimestamp() - start >= 2.0) {
                        RobotLogger.logError("Waiting for back pistons to extend timed out (2 seconds)");
                        OI.errorRumbleDriverMajor.execute();
                        OI.errorRumbleOperatorMajor.execute();
                        return;
                    }
                }
            }
        }
    }
    public State getBackState() {
        return !RobotMap.backMRS.get() ? State.EXTENDED : State.RETRACTED;
    }
    public void toggleBack() {
        toggleBack(false);
    }
    public void toggleBack(boolean wait) {
        setBackState(getBackState().opposite(), wait);
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
