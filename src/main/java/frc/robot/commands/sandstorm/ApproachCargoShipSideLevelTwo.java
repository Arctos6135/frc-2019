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

public class ApproachCargoShipSideLevelTwo extends CommandGroup {

    public ApproachCargoShipSideLevelTwo(AutoDispatcher.Side side, boolean reverse) {
        addSequential(new FollowTrajectory(reverse ? AutoPaths.driveOffHabLevelTwo.mirrorFrontBack() 
                : AutoPaths.driveOffHabLevelTwo));
        addSequential(new ApproachCargoShipSideLevelOne(side, reverse));
    }
}
