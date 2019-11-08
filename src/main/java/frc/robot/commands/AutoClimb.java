/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotlib.logging.RobotLogger;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

/**
 * Automatically does a Hab level 2 climb.
 */
public class AutoClimb extends CommandGroup {
    /**
     * Automatically does a Hab level 2 climb.
     */
    public AutoClimb() {
        // Push front side out
        addSequential(new OperateClimber(Climber.Side.ESSIE, Climber.State.EXTENDED, true));

        // Drive so that the front wheels are on the platform
        // Drive a negative distance since the front climber side is actually the back
        // robot side
        addSequential(new DriveDistance(-20));

        // Retract front pistons
        addSequential(new OperateClimber(Climber.Side.ESSIE, Climber.State.RETRACTED, false));
        // Push back side up
        addSequential(new OperateClimber(Climber.Side.HANK, Climber.State.EXTENDED, true));

        // Drive the back wheels onto the platform
        addSequential(new DriveDistance(-40));

        // Retract back pistons
        addSequential(new OperateClimber(Climber.Side.HANK, Climber.State.RETRACTED, true));

        // Drive so that the entire robot is on the platform
        addSequential(new DriveDistance(-20));

    }

    // Override these methods in order to add debug info output
    // No need to call the superclass implementations here; if you look at the
    // source, they're empty
    @Override
    protected void initialize() {
        Robot.climbingEntry.setBoolean(true);
        RobotLogger.logInfoFine("Auto climb sequence started");
    }

    @Override
    protected void interrupted() {
        Robot.climbingEntry.setBoolean(false);
        RobotLogger.logInfoFine("Auto climb sequence interrupted");
    }

    @Override
    protected void end() {
        Robot.climbingEntry.setBoolean(false);
        RobotLogger.logInfoFine("Auto climb sequence ended");
    }
}
