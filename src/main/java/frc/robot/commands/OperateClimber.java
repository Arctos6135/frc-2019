/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climber.Side;

/**
 * Operates the climber.
 */
public class OperateClimber extends Command {

    Side side;
    Climber.State state;
    boolean toggle = false;
    boolean wait;

    /**
     * Toggles one side of the climber. If wait is set to true, this command will
     * wait for the pistons to go into position before finishing.
     * 
     * @param side The side to operate
     * @param wait Whether or not to wait for the pistons
     */
    public OperateClimber(Side side, boolean wait) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
        this.state = null;
        this.toggle = true;
        this.wait = wait;
    }

    /**
     * Toggles one side of the climber.
     * 
     * @param side The side to operate
     */
    public OperateClimber(Side side) {
        this(side, false);
    }

    /**
     * Sets the state of one side of the climber.
     * 
     * @param side  The side to operate
     * @param state The state to set it to
     */
    public OperateClimber(Side side, Climber.State state) {
        this(side, state, false);
    }

    /**
     * Sets the state of one side of the climber. If wait is set to true, this
     * command will wait for the pistons to go into position before exiting.
     * 
     * @param side  The side to operate
     * @param state The state to set it to
     * @param wait  Whether or not to wait for the pistons
     */
    public OperateClimber(Side side, Climber.State state, boolean wait) {
        super();

        if(state == Climber.State.UNKNOWN) {
            throw new IllegalArgumentException("State cannot be UNKNOWN");
        }

        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
        this.state = state;
        this.wait = wait;
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        // Go into low gear
        RobotLogger.logInfoFiner("Putting robot into low gear for climbing");
        Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        if (state == null || toggle) {
            Climber.State climberState = Robot.climber.getState(side);
            if(climberState == Climber.State.UNKNOWN) {
                wait = false;
                RobotLogger.logInfoFiner("Attempting to toggle climber, but state is UNKNOWN");
                return;
            }
            state = climberState.opposite();
        }
        Robot.climber.setState(side, state);
        RobotLogger.logInfoFiner("Setting climber: " + side.toString() + " to " + state.toString());
    }

    @Override
    protected boolean isFinished() {
        if (wait) {
            if (timeSinceInitialized() >= 2.0) {
                RobotLogger.logError("Wait for climber pistons to go into position timed out");
                return true;
            }
            return Robot.climber.getState(side) == state;
        } else {
            return true;
        }
    }
}
