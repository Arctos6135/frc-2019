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
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class ApproachCargoShipFrontLevelTwo extends CommandGroup {
    
    public ApproachCargoShipFrontLevelTwo(AutoDispatcher.Side side, boolean reverse) {
        addSequential(new FollowTrajectory(reverse ? AutoPaths.driveOffHabLevelTwo.mirrorFrontBack() 
                : AutoPaths.driveOffHabLevelTwo));
        TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
        ? reverse ? AutoPaths.approachCargoShipFrontLevelOneSideL.mirrorFrontBack() : AutoPaths.approachCargoShipFrontLevelOneSideL
        : reverse ? AutoPaths.approachCargoShipFrontLevelOneSideR.mirrorFrontBack() : AutoPaths.approachCargoShipFrontLevelOneSideR;
        addSequential(new FollowTrajectory(t));
    }
}
