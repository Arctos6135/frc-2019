package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

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

    public enum Side {
        LEFT, RIGHT;
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
        return new InstantCommand();
    }
}
