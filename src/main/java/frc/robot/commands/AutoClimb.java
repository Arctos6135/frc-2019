/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Automatically does a Hab level 2 climb.
 */
public class AutoClimb extends CommandGroup {
    /**
     * Automatically does a Hab level 2 climb.
     */
    public AutoClimb() {
        // Push front side out
        addParallel(new OperateClimber(OperateClimber.Side.FRONT, DoubleSolenoid.Value.kForward));
        // Delay a bit for the piston to fully extend as we don't know when it's done
        addSequential(new Delay(0.5));

        // Drive so that the front wheels are on the platform
        // Drive a negative distance since the front climber side is actually the back robot side
        addSequential(new DriveDistance(-20));

        // Retract front pistons
        addParallel(new OperateClimber(OperateClimber.Side.FRONT, DoubleSolenoid.Value.kReverse));
        // Push back side up
        addParallel(new OperateClimber(OperateClimber.Side.BACK, DoubleSolenoid.Value.kForward));
        addSequential(new Delay(0.5));

        // Drive the back wheels onto the platform
        addSequential(new DriveDistance(-40));

        // Retract back pistons
        addParallel(new OperateClimber(OperateClimber.Side.BACK, DoubleSolenoid.Value.kReverse));
        addSequential(new Delay(0.5));

        // Drive so that the entire robot is on the platform
        addSequential(new DriveDistance(-20));
    }
}
