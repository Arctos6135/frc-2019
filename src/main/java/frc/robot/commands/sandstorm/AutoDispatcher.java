package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.AdvancedVisionAlign;
import frc.robot.commands.FollowTrajectory;
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
        switch(mode) {
        case NONE:
            return new InstantCommand();
        case DEBUG:
            return new FollowTrajectory(AutoPaths.debug);
        case FRONT:
            if(level == HabLevel.ONE) {
                // The auto is reversed if the robot side is not hank
                return new ApproachCargoShipFrontLevelOne(side, robotSide != RobotSide.HANK);
            }
            else {
                return new ApproachCargoShipFrontLevelTwo(side, robotSide != RobotSide.HANK);
            }
        case SIDE:
            if(level == HabLevel.ONE) {
                // The auto is reversed if the robot side is not hank
                return new ApproachCargoShipSideLevelOne(side, robotSide != RobotSide.HANK);
            }
            else {
                return new ApproachCargoShipSideLevelTwo(side, robotSide != RobotSide.HANK);
            }
        case VISION:
            if(robotSide == RobotSide.ESSIE) {
                return null;
            }
            if(level == HabLevel.ONE) {
                return new AdvancedVisionAlign();
            }
            else {
                return new ApproachCargoShipVisionLevelTwo();
            }
        case SIDE_VISION:
            if(robotSide == RobotSide.ESSIE) {
                return null;
            }
            if(level == HabLevel.ONE) {
                return new ApproachCargoShipSideVisionLevelOne(side);
            }
            else {
                return new ApproachCargoShipSideVisionLevelTwo(side);
            }
        default: return null;
        }
    }
}
