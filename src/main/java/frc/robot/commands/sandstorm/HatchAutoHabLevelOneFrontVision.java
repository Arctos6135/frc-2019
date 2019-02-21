/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.AdvancedVisionAlign;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.OperateHank;
import frc.robot.misc.AutoPaths;

/**
 * Hatch auto that starts from hab level 1 using vision to align.
 */
public class HatchAutoHabLevelOneFrontVision extends CommandGroup {

    public HatchAutoHabLevelOneFrontVision() {
        addSequential(new AdvancedVisionAlign());
        addSequential(new OperateHank());
        addSequential(new FollowTrajectory(AutoPaths.driveBack));
    }
}
