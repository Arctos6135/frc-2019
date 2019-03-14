/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.follower.Follower.DirectionSource;
import com.arctos6135.robotpathfinder.follower.Follower.DistanceSource;
import com.arctos6135.robotpathfinder.follower.Follower.Motor;
import com.arctos6135.robotpathfinder.follower.Follower.TimestampSource;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends Command {

    public static final Motor L_MOTOR = Robot.drivetrain::setLeftMotor;
    public static final Motor R_MOTOR = Robot.drivetrain::setRightMotor;
    public static final DirectionSource GYRO = () -> {
        return Math.toRadians(Robot.drivetrain.getHeading());
    };
    public static final DistanceSource L_DISTANCE_SOURCE = Robot.drivetrain::getLeftDistance;
    public static final DistanceSource R_DISTANCE_SOURCE = Robot.drivetrain::getRightDistance;
    public static final TimestampSource TIMESTAMP_SOURCE = Timer::getFPGATimestamp;

    public static double kP_l = 0.1, kD_l = 0.00025, kV_l = 0.025, kA_l = 0.002, kDP_l = 0.01;
    public static double kP_h = 0.07, kD_h = 0.001, kV_h = 0.007, kA_h = 0.002, kDP_h = 0.01;

    // This is the gear the robot must be in for trajectory following
    // If set to null, the robot will accept both
    public static Drivetrain.Gear gearToUse = null;

    public final TankDriveTrajectory trajectory;
    public TankDriveFollower follower;

    public FollowTrajectory(TankDriveTrajectory trajectory) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        this.trajectory = trajectory;
    }

    private Drivetrain.Gear startingGear;

    // Called just before this Command runs the first time
    // Note we made this method public! This is so that Commands that wrap around this one have an easier time.
    @Override
    public void initialize() {
        RobotLogger.logInfoFine("FollowTrajectory started");
        Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
        // If the gear to use is not null, make sure the robot is in the correct gear
        if(gearToUse != null) {
            startingGear = Robot.drivetrain.getGear();
            Robot.drivetrain.setGear(gearToUse);
        }

        if(Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH) {
            follower = new TankDriveFollower(trajectory, L_MOTOR, R_MOTOR, L_DISTANCE_SOURCE, R_DISTANCE_SOURCE, TIMESTAMP_SOURCE, 
                    GYRO, kV_h, kA_h, kP_h, kD_h, kDP_h);
        }
        else {
            follower = new TankDriveFollower(trajectory, L_MOTOR, R_MOTOR, L_DISTANCE_SOURCE, R_DISTANCE_SOURCE, TIMESTAMP_SOURCE, 
                    GYRO, kV_l, kA_l, kP_l, kD_l, kDP_l);
        }

        follower.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
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
    public boolean isFinished() {
        return !follower.isRunning();
    }

    // Called once after isFinished returns true
    @Override
    public void end() {
        follower.stop();
        Robot.drivetrain.setMotors(0, 0);
        
        if(gearToUse != null) {
            Robot.drivetrain.setGear(startingGear);
        }

        RobotLogger.logInfoFine("FollowTrajectory ended");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    public void interrupted() {
        follower.stop();
        Robot.drivetrain.setMotors(0, 0);

        if(gearToUse != null) {
            Robot.drivetrain.setGear(startingGear);
        }

        RobotLogger.logInfoFine("FollowTrajectory interrupted");
    }

    /**
     * Retrieves the correct {@link RobotSpecs} based on the gear to use in autos and/or the robot's current gear.
     * @return The correct {@link RobotSpecs}
     */
    public static RobotSpecs getSpecs() {
        // If the gear to use in autos is specified, generate trajectories based on that
        if(gearToUse != null) {
            return gearToUse == Drivetrain.Gear.HIGH ? RobotMap.specsHigh : RobotMap.specsLow;
        }
        else {
            // Otherwise generate it based on the robot's current gear
            return Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH ? RobotMap.specsHigh : RobotMap.specsLow;
        }
    }
}
