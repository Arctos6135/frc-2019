/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Operates Hank.
 */
public class OperateHank extends Command {

    /**
     * Toggles Hank.
     */
    public OperateHank() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.hank);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.hank.pushOut();
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return timeSinceInitialized() > 0.6;
    }

    @Override
    protected void end() {
        Robot.hank.retract();
    }

    @Override
    protected void interrupted() {
        Robot.hank.retract();
    }
}
