/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.misc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import robot.pathfinder.follower.Follower.Motor;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.follower.TankFollower;
import robot.pathfinder.follower.Follower.DirectionSource;
import robot.pathfinder.follower.Follower.TimestampSource;
import robot.pathfinder.follower.Follower.DistanceSource;

public class FollowTrajectory extends Command {

    public static final Motor L_MOTOR = (speed) -> {
        Robot.drivetrain.setLeftMotor(speed);
    };
    public static final Motor R_MOTOR = (speed) -> {
        Robot.drivetrain.setRightMotor(speed);
    };
    public static final DirectionSource GYRO = () -> {
        return Robot.drivetrain.getHeading();
    };
    public static final DistanceSource L_DISTANCE_SOURCE = () -> {
        return Robot.drivetrain.getLeftDistance();
    };
    public static final DistanceSource R_DISTANCE_SOURCE = () -> {
        return Robot.drivetrain.getRightDistance();
    };
    public static final TimestampSource TIMESTAMP_SOURCE = () -> {
        return Timer.getFPGATimestamp();
    };

    public static double kP = 0, kD = 0, kV = 0, kA = 0, kDP = 0;

    final TankFollower follower;

    public FollowTrajectory(TankDriveTrajectory trajectory) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        follower = new TankFollower(trajectory, L_MOTOR, R_MOTOR, L_DISTANCE_SOURCE, R_DISTANCE_SOURCE, TIMESTAMP_SOURCE, 
                GYRO, kV, kA, kP, kD, kDP);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        follower.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        follower.run();

        if(Robot.isInDebugMode) {
            SmartDashboard.putNumber("Follower Left Output", follower.lastLeftOutput());
            SmartDashboard.putNumber("Follower Right Output", follower.lastRightOutput());

            SmartDashboard.putNumber("Follower Left Velocity", follower.lastLeftVelocity());
            SmartDashboard.putNumber("Follower Right Velocity", follower.lastRightVelocity());

            SmartDashboard.putNumber("Follower Left Acceleration", follower.lastLeftAcceleration());
            SmartDashboard.putNumber("Follower Right Acceleration", follower.lastRightAcceleration());

            SmartDashboard.putNumber("Follower Left Error", follower.lastLeftError());
            SmartDashboard.putNumber("Follower Right Error", follower.lastRightError());
            
            SmartDashboard.putNumber("Follower Left Error Derivative", follower.lastLeftDerivative());
            SmartDashboard.putNumber("Follower Right Error Derivative", follower.lastRightDerivative());

            SmartDashboard.putNumber("Follower Directional Error", follower.lastDirectionalError());
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return !follower.isRunning();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        follower.stop();
        Robot.drivetrain.setMotors(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        follower.stop();
        Robot.drivetrain.setMotors(0, 0);
    }
}
