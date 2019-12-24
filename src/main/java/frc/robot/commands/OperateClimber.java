/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.Side;
import frc.robot.subsystems.Drivetrain;

/**
 * Operates the climber.
 */
public class OperateClimber extends CommandBase {

    private Side side;
    private Climber.State state;
    private boolean toggle = false;
    private boolean wait;

    private Timer timer = new Timer();

    /**
     * Toggles one side of the climber. If wait is set to true, this command will
     * wait for the pistons to go into position before finishing.
     * 
     * @param side The side to operate
     * @param wait Whether or not to wait for the pistons
     */
    public OperateClimber(Side side, boolean wait) {
        addRequirements(Robot.climber, Robot.drivetrain);
        
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
        if(state == Climber.State.UNKNOWN) {
            throw new IllegalArgumentException("State cannot be UNKNOWN");
        }

        addRequirements(Robot.climber, Robot.drivetrain);
        this.side = side;
        this.state = state;
        this.wait = wait;
    }

    // Called once when the command executes
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // Go into low gear
        Robot.logger.logInfoFiner("Putting robot into low gear for climbing");
        Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        if (state == null || toggle) {
            Climber.State climberState = Robot.climber.getState(side);
            if(climberState == Climber.State.UNKNOWN) {
                wait = false;
                Robot.logger.logInfoFiner("Attempting to toggle climber, but state is UNKNOWN");
                return;
            }
            state = climberState.opposite();
        }
        Robot.climber.setState(side, state);
        Robot.logger.logInfoFiner("Setting climber: " + side.toString() + " to " + state.toString());
    }

    @Override
    public boolean isFinished() {
        if (wait) {
            if (timer.get() >= 2.0) {
                Robot.logger.logError("Wait for climber pistons to go into position timed out");
                return true;
            }
            return Robot.climber.getState(side) == state;
        } else {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
