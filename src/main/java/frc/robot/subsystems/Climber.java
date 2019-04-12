/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.misc.RobotLogger;

/**
 * Controls Hab 2 climber pistons.
 */
public class Climber extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * Enum for the two states of a climber piston, extended or retracted.
     */
    public enum State {
        EXTENDED, RETRACTED, UNKNOWN;

        /**
         * @throws IllegalArgumentException If the value is UNKNOWN
         */
        public boolean value() {
            if (this == EXTENDED) {
                return true;
            } else if (this == RETRACTED) {
                return false;
            } else {
                throw new IllegalArgumentException("State is unknown");
            }
        }
        /**
         * @throws IllegalArgumentException If the value is UNKNOWN
         */
        public State opposite() {
            if (this == EXTENDED) {
                return RETRACTED;
            } else if (this == RETRACTED) {
                return EXTENDED;
            } else {
                throw new IllegalArgumentException("State is unknown");
            }
        }
    }

    public static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            // Ignore
        }
    }

    /**
     * Enum for the two piston sides.
     */
    public enum Side {
        ESSIE, HANK;
    }

    public State getState(Side side) {
        if(side == Side.ESSIE) {
            return !RobotMap.essieDownMRS.get() ? State.EXTENDED : (!RobotMap.essieUpMRS.get() ? State.RETRACTED : State.UNKNOWN);
        }
        else {
            return !RobotMap.hankDownMRS.get() ? State.EXTENDED : (!RobotMap.hankUpMRS.get() ? State.RETRACTED : State.UNKNOWN);
        }
    }

    /**
     * @throws IllegalArgumentException If state is UNKNOWN
     */
    public void setState(Side side, State state) {
        setState(side, state, false);
    }
    /**
     * @throws IllegalArgumentException If state is UNKNOWN
     */
    public void setState(Side side, State state, boolean wait) {
        if(state == State.UNKNOWN) {
            throw new IllegalArgumentException("State cannot be UNKNOWN");
        }
        RobotLogger.logInfoFine("Setting " + side.toString() + " climbers to " + state.toString() + " wait=" + wait);
        @SuppressWarnings("resource")
        Solenoid climber = side == Side.ESSIE ? RobotMap.essieClimber : RobotMap.hankClimber;

        climber.set(state.value());

        if (wait) {
            double start = Timer.getFPGATimestamp();
            while (getState(side) != state) {
                if (Timer.getFPGATimestamp() - start >= 2.0) {
                    RobotLogger.logError("Waiting for front pistons to extend timed out (2 seconds)");
                    OI.errorRumbleDriverMajor.execute();
                    OI.errorRumbleOperatorMajor.execute();
                    return;
                }
                sleep(50);
            }
        }
    }

    public void toggle(Side side) {
        toggle(side, false);
    }

    public void toggle(Side side, boolean wait) {
        if(getState(side) != State.UNKNOWN) {
            setState(side, getState(side).opposite(), wait);
        }
    }

    public Climber() {
        super();
        setState(Side.ESSIE, State.RETRACTED);
        setState(Side.HANK, State.RETRACTED);
    }

    public Climber(String name) {
        super(name);
        setState(Side.ESSIE, State.RETRACTED);
        setState(Side.HANK, State.RETRACTED);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
