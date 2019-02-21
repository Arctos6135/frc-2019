/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.OperateHank;
import frc.robot.misc.AutoPaths;

/**
 * Hatch auto that starts from hab level 1 aligned with the cargo ship.
 */
public class HatchAutoHabLevelOneFrontAligned extends CommandGroup {
    /**
     * Add your docs here.
     */
    public HatchAutoHabLevelOneFrontAligned() {
        addSequential(new FollowTrajectory(AutoPaths.hatchAutoHabLevel1AlignedFront));
        addSequential(new OperateHank());
        addSequential(new FollowTrajectory(AutoPaths.driveBack));
    }
}
