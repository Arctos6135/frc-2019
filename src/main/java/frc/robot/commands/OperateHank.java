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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * This command operates Hank. When started, it pushes Hank out, and when
 * ending/interrupted, it retracts Hank.
 * 
 * <b>Note that this command will never terminate on its own.</b> As a result,
 * it must be manually interrupted, or used with the
 * {@link Button#whileHeld(Command)} or {@link Trigger#whileActive(Command)}.
 */
public class OperateHank extends CommandBase {

    /**
     * This command operates Hank. When started, it pushes Hank out, and when
     * ending/interrupted, it retracts Hank.
     * 
     * <b>Note that this command will never terminate on its own.</b> As a result,
     * it must be manually interrupted, or used with the
     * {@link Button#whileHeld(Command)} or {@link Trigger#whileActive(Command)}.
     */
    public OperateHank() {
        addRequirements(Robot.hank);
    }

    // Called once when the command executes
    @Override
    public void initialize() {
        Robot.logger.logInfoFine("Hank operation started");
        Robot.hank.pushOut();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.logger.logInfoFine(interrupted ? "Hank operation interrupted" : "Hank operation ended");
        Robot.hank.retract();
    }
}
