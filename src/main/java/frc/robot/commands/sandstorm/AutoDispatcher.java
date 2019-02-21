package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.FollowTrajectory;
import frc.robot.misc.AutoPaths;

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
            return new FollowTrajectory(AutoPaths.debug);
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
