package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.commands.FollowTrajectory;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

/**
 * Finds the correct autonomous command based on parameters.
 */
public final class AutoDispatcher {

    /**
     * Levels of the hab.
     */
    public enum HabLevel {
        ONE, TWO, THREE;
    }

    /**
     * For auto choosing.
     * Left, right aligned/middle or vision, plus other special options.
     */
    public enum Mode {
        NONE, LEFT, RIGHT, ALIGNED, VISION, DEBUG;
    }

    /**
     * This checked exception is thrown by {@link AutoDispatcher#getAuto(HabLevel, Mode) } if no auto matches the options.
     */
    public static final class AutoNotFoundException extends Exception {
        private static final long serialVersionUID = -4080978647629416633L;

        public AutoNotFoundException() {
            super();
        }
        public AutoNotFoundException(String msg) {
            super(msg);
        }
    }

    /**
     * Gets the corresponding auto command based on parameters.
     * @param level The hab level to start on
     * @param mode The side of the auto, or aligned, or vision
     * @return The corresponding auto
     * @throws AutoNotFoundException If no matching auto command is found
     */
    public static final Command getAuto(HabLevel level, Mode mode) throws AutoNotFoundException {
        if(mode == Mode.NONE) {
            // Return an empty InstantCommand for no auto.
            return new InstantCommand();
        }
        // Debug auto
        else if(mode == Mode.DEBUG) {
            TrajectoryParams params = new TrajectoryParams();
            params.waypoints = new Waypoint[] {
                new Waypoint(0.0, 0.0, Math.PI / 2),
                new Waypoint(60.0, 120.0, Math.PI / 2),
            };
            params.alpha = 150.0;
            params.segmentCount = 500;
            params.isTank = true;
            params.pathType = PathType.QUINTIC_HERMITE;
            TankDriveTrajectory trajectory = new TankDriveTrajectory(RobotMap.specs, params);
            return new FollowTrajectory(trajectory);
        }
        switch(level) {
        case ONE:
            switch(mode) {
            case ALIGNED:
                return new HatchAutoHabLevelOneFrontAligned();
            case LEFT:
                return new HatchAutoHabLevelOneSide(mode);
            case RIGHT:
                return new HatchAutoHabLevelOneSide(mode);
            case VISION:
                return new HatchAutoHabLevelOneFrontVision();
            default:
                throw new AutoNotFoundException("No auto found");
            }
        case TWO:
            switch(mode) {
            case VISION: 
                return new HatchAutoHabLevelTwoFrontVision();
            default: throw new AutoNotFoundException("No auto found");
            }
        case THREE:
            throw new AutoNotFoundException("No autos for hab level 3");
        default:
            throw new AutoNotFoundException("No auto found");
        }
    }
}
