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
    /**
      * Add your docs here.
      */
    public ToggleClimber() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.climberPistons);
        requires(Robot.drivetrain);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        // When extending, go into low gear
        if(Robot.climberPistons.getState() == DoubleSolenoid.Value.kReverse) {
            RobotLogger.logInfoFiner("Putting robot into low gear for climbing");
            Robot.drivetrain.setGear(Drivetrain.Gear.LOW);
        }
        Robot.climberPistons.toggle();
        RobotLogger.logInfoFiner("Climber pistons toggled to " + Robot.climberPistons.getState().toString());
    }

}
