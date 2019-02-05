package frc.robot.commands.sandstorm;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Finds the correct autonomous command based on parameters.
 */
public final class AutoDispatcher {
    /**
     * This checked exception is thrown by {@link AutoDispatcher#getAuto(HabLevel, Side) } if no auto matches the options.
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
     * @param side The side of the auto, or aligned, or vision
     * @return The corresponding auto
     * @throws AutoNotFoundException If no matching auto command is found
     */
    public static final Command getAuto(HabLevel level, Side side) throws AutoNotFoundException {
        switch(level) {
        case ONE:
            switch(side) {
            case ALIGNED:
                return new HatchAutoHabLevelOneFrontAligned();
            case LEFT:
                return new HatchAutoHabLevelOneSide(side);
            case RIGHT:
                return new HatchAutoHabLevelOneSide(side);
            case VISION:
                return new HatchAutoHabLevelOneFrontVision();
            default:
                throw new AutoNotFoundException("No auto found");
            }
        case TWO:
            throw new AutoNotFoundException("Not implemented");
        case THREE:
            throw new AutoNotFoundException("No autos for hab level 3");
        default:
            throw new AutoNotFoundException("No auto found");
        }
    }
}
