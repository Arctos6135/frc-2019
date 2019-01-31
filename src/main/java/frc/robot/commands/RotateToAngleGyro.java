/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.subsystems.Drivetrain.constrainAngle;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Uses the navX gyro reading to turn the robot a certain amount of degrees.
 */
public class RotateToAngleGyro extends Command {

    public enum Direction {
        LEFT, RIGHT;
    }

    private double angle;
    private final Direction direction;
    private final double speed;
    
    private double startingHeading;

    public RotateToAngleGyro(double angle, Direction direction) {
        requires(Robot.drivetrain);
        // Left is positive
        this.angle = angle;
        this.direction = direction;
        speed = 1.0;
    }

    public RotateToAngleGyro(double angle, Direction direction, double speed) {
        requires(Robot.drivetrain);
        // Left is positive
        this.angle = angle;
        this.speed = speed;
        this.direction = direction;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    private NeutralMode originalMode;

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // Set the drive motors into brake mode
        originalMode = Robot.drivetrain.getNeutralMode();
        Robot.drivetrain.setNeutralMode(NeutralMode.Brake);

        if(direction == Direction.LEFT) {
            Robot.drivetrain.setMotors(-speed, speed);
        }
        else {
            Robot.drivetrain.setMotors(speed, -speed);
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // The heading from the navX may need to be reversed
        if(direction == Direction.LEFT) {
            return constrainAngle(Robot.drivetrain.getHeading() - startingHeading) >= angle;
        }
        else {
            return constrainAngle(Robot.drivetrain.getHeading() - startingHeading) <= -angle;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drivetrain.setMotors(0, 0);
        Robot.drivetrain.setNeutralMode(originalMode);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
