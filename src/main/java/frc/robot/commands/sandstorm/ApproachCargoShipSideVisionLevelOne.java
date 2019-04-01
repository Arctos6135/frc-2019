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
import frc.robot.misc.AutoPaths;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class ApproachCargoShipSideVisionLevelOne extends CommandGroup {
    public ApproachCargoShipSideVisionLevelOne(AutoDispatcher.Side side) {
        TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
                ? AutoPaths.approachCargoShipSideForVisionLevelOneL
                : AutoPaths.approachCargoShipSideForVisionLevelOneR;
        addSequential(new FollowTrajectory(t));
        addSequential(new AdvancedVisionAlign());
    }
}
