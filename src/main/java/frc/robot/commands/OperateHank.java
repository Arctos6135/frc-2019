/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;

/**
 * This command operates Hank. When started, it pushes Hank out, and when ending/interrupted, it retracts Hank.
 * 
 * <b>Note that this command will never terminate on its own.</b>
 * As a result, it must be manually interrupted, or used with the {@link Button#whileHeld(Command)} or 
 * {@link Trigger#whileActive(Command)}.
 */
public class OperateHank extends Command {

    /**
     * This command operates Hank. When started, it pushes Hank out, and when ending/interrupted, it retracts Hank.
     * 
     * <b>Note that this command will never terminate on its own.</b>
     * As a result, it must be manually interrupted, or used with the {@link Button#whileHeld(Command)} or 
     * {@link Trigger#whileActive(Command)}.
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
        RobotLogger.logInfoFine("Hank operation started");
        Robot.hank.pushOut();
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        RobotLogger.logInfoFine("Hank operation ended");
        Robot.hank.retract();
    }

    @Override
    protected void interrupted() {
        RobotLogger.logInfoFine("Hank operation interrupted");
        Robot.hank.retract();
    }
}
