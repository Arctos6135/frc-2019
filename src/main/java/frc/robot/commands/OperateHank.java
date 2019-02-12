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

    final double timeout;

    /**
     * Toggles Hank.
     */
    public OperateHank() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.hank);
        timeout = 0.6;
    }

    public OperateHank(double timeout) {
        super();

        requires(Robot.hank);
        this.timeout = timeout;
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
        return timeSinceInitialized() > timeout;
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
