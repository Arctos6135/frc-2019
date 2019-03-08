/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

/**
  * Operates the climber.
  */
public class OperateClimber extends InstantCommand {

    final Side side;
    final Climber.State state;
    
    public OperateClimber(Side side) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
        this.state = null;
    }

    public OperateClimber(Side side, Climber.State state) {
        super();
        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
        this.state = state;
    }

    public enum Side {
        FRONT, BACK;
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        // When extending, go into low gear
        RobotLogger.logInfoFiner("Putting robot into low gear for climbing");
        Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        if(state == null) {
            if(side == Side.FRONT) {
                Robot.climber.toggleFront();
                RobotLogger.logInfoFiner("Front climber pistons toggled to " + Robot.climber.getFrontState().toString());
            }
            else {
                Robot.climber.toggleBack();
                RobotLogger.logInfoFiner("Back climber pistons toggled to " + Robot.climber.getBackState().toString());
            }
        }
        else {
            if(side == Side.FRONT) {
                Robot.climber.setFrontState(state);
                RobotLogger.logInfoFiner("Front climber pistons set to " + state.toString());
            }
            else {
                Robot.climber.setBackState(state);
                RobotLogger.logInfoFiner("Back climber pistons set to " + state.toString());
            }
        }
    }

}
