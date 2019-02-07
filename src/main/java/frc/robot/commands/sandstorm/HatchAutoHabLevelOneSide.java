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
import frc.robot.commands.sandstorm.AutoDispatcher.Mode;
import frc.robot.misc.AutoPaths;

public class HatchAutoHabLevelOneSide extends CommandGroup {

    public HatchAutoHabLevelOneSide(Mode side) {
        addSequential(new FollowTrajectory(side == Mode.LEFT ? AutoPaths.hatchAutoHabLevel1SideLeft : AutoPaths.hatchAutoHabLevel1SideRight));
        addSequential(new OperateHank());
        addSequential(new FollowTrajectory(AutoPaths.driveBack));
    }
}
