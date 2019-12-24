package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * Contains methods that create command groups.
 */
public final class CommandGroups {

    private CommandGroups() {
    }

    /**
     * Creates an auto climb command (climbs hab 2 automatically).
     * 
     * <p>
     * The returned command will return "AutoClimb" when {@link Command#getName()}
     * is called.
     * </p>
     * 
     * @return The auto climb command.
     */
    public static Command autoClimb() {

        return new SequentialCommandGroup(new OperateClimber(Climber.Side.ESSIE, Climber.State.EXTENDED, true),
                new DriveDistance(-20),
                new OperateClimber(Climber.Side.ESSIE, Climber.State.RETRACTED, false).alongWith(
                        new OperateClimber(Climber.Side.HANK, Climber.State.EXTENDED, true)),
                new DriveDistance(-40), new OperateClimber(Climber.Side.HANK, Climber.State.RETRACTED, true),
                new DriveDistance(-20)) {
            @Override
            public String getName() {
                return "AutoClimb";
            }
        };
    }
}
