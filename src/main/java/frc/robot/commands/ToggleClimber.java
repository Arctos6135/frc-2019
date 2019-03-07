/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Drivetrain;

/**
  * Add your docs here.
  */
public class ToggleClimber extends InstantCommand {

    final Side side;
    /**
      * Add your docs here.
      */
    public ToggleClimber(Side side) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.climber);
        requires(Robot.drivetrain);
        this.side = side;
    }

    public enum Side {
        FRONT, BACK;
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        // When extending, go into low gear
        if(Robot.climber.getFrontState() == DoubleSolenoid.Value.kReverse) {
            RobotLogger.logInfoFiner("Putting robot into low gear for climbing");
            Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        }
        if(side == Side.FRONT) {
            Robot.climber.toggleFront();
            RobotLogger.logInfoFiner("Front climber pistons toggled to " + Robot.climber.getFrontState().toString());
        }
        else {
            Robot.climber.toggleBack();
            RobotLogger.logInfoFiner("Back climber pistons toggled to " + Robot.climber.getBackState().toString());
        }
    }

}
