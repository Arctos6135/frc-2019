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
    boolean wait;

    public OperateClimber(Side side, boolean wait) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
        this.state = null;
        this.wait = wait;
    }

    public OperateClimber(Side side) {
        this(side, false);
    }

    public OperateClimber(Side side, Climber.State state, boolean wait) {
        super();
        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
        this.state = state;
        this.wait = wait;
    }

    public OperateClimber(Side side, Climber.State state) {
        this(side, state, false);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        // When extending, go into low gear
        RobotLogger.logInfoFiner("Putting robot into low gear for climbing");
        Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        if (state == null) {
            if (side == Side.FRONT) {
                state = Robot.climber.getFrontState().opposite();
                Robot.climber.toggleFront();
            } else {
                state = Robot.climber.getBackState().opposite();
                Robot.climber.toggleBack();
            }
        } else {
            if (side == Side.FRONT) {
                Robot.climber.setFrontState(state);
            } else {
                Robot.climber.setBackState(state);
            }
        }
    }

    @Override
    protected boolean isFinished() {
        if (wait) {
            if (state == Climber.State.RETRACTED) {
                return timeSinceInitialized() >= 0.5;
            } else {
                Climber.State s = side == Side.FRONT ? Robot.climber.getFrontState() : Robot.climber.getBackState();
                return s == state || timeSinceInitialized() >= 0.5;
            }
        } else {
            return true;
        }
    }
}
