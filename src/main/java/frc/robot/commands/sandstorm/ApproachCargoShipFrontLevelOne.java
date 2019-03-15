/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sandstorm;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.FollowTrajectory;
import frc.robot.misc.AutoPaths;

public class ApproachCargoShipFrontLevelOne extends CommandGroup {

    public ApproachCargoShipFrontLevelOne(AutoDispatcher.Side side, boolean reverse) {
        /*
         * As you may be able to tell, here we have a case of a memory leak (or at least bad memory management)
         * due to the RobotPathfinder Native object being created but never freed.
         * Unfortunately this issue had to be ignored. The main reasons being:
         * * This constructor is likely only going to be called once in a single match.
         * * There is no suitable alternative (everything else would use even more memory).
         * * The GlobalLifeCycleManager will eventually free the native resources, though it is inefficient.
         */
        TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
                ? reverse ? AutoPaths.approachCargoShipFrontLevelOneL.mirrorFrontBack() : AutoPaths.approachCargoShipFrontLevelOneL
                : reverse ? AutoPaths.approachCargoShipFrontLevelOneR.mirrorFrontBack() : AutoPaths.approachCargoShipFrontLevelOneR;
        addSequential(new FollowTrajectory(t));
    }
}
