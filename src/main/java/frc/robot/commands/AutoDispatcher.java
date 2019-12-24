package frc.robot.commands;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.misc.AutoPaths;

/**
 * Finds the correct autonomous command based on parameters.
 */
public final class AutoDispatcher {

    /**
     * Levels of the hab to start on.
     */
    public enum HabLevel {
        ONE, TWO;
    }

    /**
     * Auto mode.
     */
    public enum Mode {
        NONE, FRONT, SIDE, VISION, SIDE_VISION, DEBUG;
    }

    /**
     * Left or right side.
     */
    public enum Side {
        LEFT, RIGHT;
    }

    /**
     * Hank side or Essie side of the robot.
     */
    public enum RobotSide {
        HANK, ESSIE;
    }

    public static final Command getAuto(Mode mode, HabLevel level, Side side, RobotSide robotSide) {
        switch (mode) {
        case NONE:
            return new InstantCommand();
        case DEBUG:
            return new DriveDistance(120);
        case FRONT: {
            boolean reverse = robotSide != RobotSide.HANK;
            if (level == HabLevel.ONE) {
                /*
                 * As you may be able to tell, here we have a case of a memory leak (or at least
                 * bad memory management) due to the RobotPathfinder Native object being created
                 * but never freed. Unfortunately this issue had to be ignored. The main reasons
                 * being: * This constructor is likely only going to be called once in a single
                 * match. * There is no suitable alternative (everything else would use even
                 * more memory). * The GlobalLifeCycleManager will eventually free the native
                 * resources, though it is inefficient.
                 */
                TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
                        ? reverse ? AutoPaths.approachCargoShipFrontLevelOneL.mirrorFrontBack()
                                : AutoPaths.approachCargoShipFrontLevelOneL
                        : reverse ? AutoPaths.approachCargoShipFrontLevelOneR.mirrorFrontBack()
                                : AutoPaths.approachCargoShipFrontLevelOneR;

                return new FollowTrajectory(t);
            } else {
                TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
                        ? reverse ? AutoPaths.approachCargoShipFrontLevelOneSideL.mirrorFrontBack()
                                : AutoPaths.approachCargoShipFrontLevelOneSideL
                        : reverse ? AutoPaths.approachCargoShipFrontLevelOneSideR.mirrorFrontBack()
                                : AutoPaths.approachCargoShipFrontLevelOneSideR;

                return new FollowTrajectory(
                        reverse ? AutoPaths.driveOffHabLevelTwoReversed : AutoPaths.driveOffHabLevelTwo)
                                .andThen(new FollowTrajectory(t));
            }
        }
        case SIDE: {
            boolean reverse = robotSide != RobotSide.HANK;
            TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT
                    ? reverse ? AutoPaths.approachCargoShipSideLevelOneL.mirrorFrontBack()
                            : AutoPaths.approachCargoShipSideLevelOneL
                    : reverse ? AutoPaths.approachCargoShipSideLevelOneR.mirrorFrontBack()
                            : AutoPaths.approachCargoShipSideLevelOneR;
            if (level == HabLevel.ONE) {
                return new FollowTrajectory(t);

            } else {
                return new FollowTrajectory(
                        reverse ? AutoPaths.driveOffHabLevelTwoReversed : AutoPaths.driveOffHabLevelTwo)
                                .andThen(new FollowTrajectory(t));
            }
        }
        case VISION:
            if (robotSide == RobotSide.ESSIE) {
                return null;
            }
            if (level == HabLevel.ONE) {
                return new AdvancedVisionAlign();
            } else {
                return new FollowTrajectory(AutoPaths.driveOffHabLevelTwo).andThen(new AdvancedVisionAlign());
            }
        case SIDE_VISION: {
            if (robotSide == RobotSide.ESSIE) {
                return null;
            }
            TankDriveTrajectory t = side == AutoDispatcher.Side.LEFT ? AutoPaths.approachCargoShipSideForVisionLevelOneL
                    : AutoPaths.approachCargoShipSideForVisionLevelOneR;
            if (level == HabLevel.ONE) {
                return new FollowTrajectory(t).andThen(new AdvancedVisionAlign());
            } else {
                return new FollowTrajectory(AutoPaths.driveOffHabLevelTwo)
                        .andThen(new FollowTrajectory(t).andThen(new AdvancedVisionAlign()));
            }
        }
        default:
            return null;
        }
    }
}
