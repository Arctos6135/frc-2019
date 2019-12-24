/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * This command operates Essie. When started, it operates Essie in the mode
 * specified, and when stopped/interrupted, it will stop Essie.
 * 
 * <b>Note that this command will never terminate on its own.</b> As a result,
 * it must be manually interrupted, or used with the
 * {@link Button#whileHeld(Command)} or {@link Trigger#whileActive(Command)}.
 */
public class OperateEssie extends CommandBase {

    public enum Mode {
        OUT_LOW, OUT_HIGH, REVERSE;
    }

    private final Mode mode;

    /**
     * This command operates Essie. When started, it operates Essie in the mode
     * specified, and when stopped/interrupted, it will stop Essie.
     * 
     * <b>Note that this command will never terminate on its own.</b> As a result,
     * it must be manually interrupted, or used with the
     * {@link Button#whileHeld(Command)} or {@link Trigger#whileActive(Command)}.
     * 
     * @param mode The mode to operate Essie in
     */
    public OperateEssie(Mode mode) {
        addRequirements(Robot.essie);
        this.mode = mode;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Robot.logger.logInfoFine("Essie operation started with mode " + mode.toString());
        switch (mode) {
        case OUT_LOW:
            Robot.essie.startOuttakeLow();
            break;
        case OUT_HIGH:
            Robot.essie.startOuttakeHigh();
            break;
        case REVERSE:
            Robot.essie.reverseIntake();
            break;
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Robot.logger.logInfoFine(interrupted ? "Essie operation interrupted" : "Essie operation ended");
        Robot.essie.stop();
    }
}
