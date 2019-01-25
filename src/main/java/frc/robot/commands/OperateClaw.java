/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;

/**
 * Operates the claw. This command either toggles it, or sets it to a specific state.
 */
public class OperateClaw extends InstantCommand {

    Claw.State state = null;
    /**
     * Toggles the state of the claw.
     */
    public OperateClaw() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.claw);
    }
    /**
     * Opens or closes the claw.
     * @param state The state to set the claw to
     */
    public OperateClaw(Claw.State state) {
        super();
        requires(Robot.claw);
        this.state = state;
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        if(state != null) {
            Robot.claw.set(state);
            return;
        }
        Robot.claw.set(Robot.claw.getState() == Claw.State.OPEN ? Claw.State.CLOSED : Claw.State.OPEN);
    }

}
