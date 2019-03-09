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

public class ApproachCargoShipFrontLevelOne extends CommandGroup {

    public ApproachCargoShipFrontLevelOne(AutoDispatcher.Side side, boolean reverse) {
        TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
                ? reverse ? AutoPaths.approachCargoShipFrontLevelOneL.mirrorFrontBack() : AutoPaths.approachCargoShipFrontLevelOneL
                : reverse ? AutoPaths.approachCargoShipFrontLevelOneR.mirrorFrontBack() : AutoPaths.approachCargoShipFrontLevelOneR;
        addSequential(new FollowTrajectory(t));
    }
}
