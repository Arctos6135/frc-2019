/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.FollowTrajectory;
import frc.robot.misc.AutoPaths;

/**
 * Hatch auto that starts from hab level 2 using vision to align.
 */
public class HatchAutoHabLevelTwoFrontVision extends CommandGroup {

    public HatchAutoHabLevelTwoFrontVision() {
        addSequential(new FollowTrajectory(AutoPaths.dropFromHabLevel2));
        addSequential(new HatchAutoHabLevelOneFrontVision());
    }
}
