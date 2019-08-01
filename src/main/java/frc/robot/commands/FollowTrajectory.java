/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicTankDriveFollower;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.Follower.AdvancedPositionSource;
import com.arctos6135.robotpathfinder.follower.Follower.DirectionSource;
import com.arctos6135.robotpathfinder.follower.Follower.Motor;
import com.arctos6135.robotpathfinder.follower.Follower.TimestampSource;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.Drivetrain;

/**
 * A {@link Command} that follows any RobotPathfinder {@link Followable
 * Followable&lt;TankDriveMoment&gt;}.
 * 
 * If the profile provided for following is an instance of
 * {@link DynamicFollowable}, then this Command will automatically use a
 * {@link DynamicTankDriveFollower} instead of a regular
 * {@link TankDriveFollower}.
 */
public class FollowTrajectory extends Command {

    /**
     * An implementation of {@link AdvancedPositionSource} using an {@link Encoder}.
     */
    private static class EncoderAdvancedPositionSource implements AdvancedPositionSource {

        protected Encoder encoder;

        public EncoderAdvancedPositionSource(Encoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getPosition() {
            return encoder.getDistance();
        }

        @Override
        public double getVelocity() {
            return encoder.getRate();
        }

        @Override
        public double getAcceleration() {
            // Maintenance Note: Because in the current version of RobotPathfinder, none of
            // the dynamic motion profiles actually require a correct acceleration to update
            // (since they're all trapezoidal anyways), we can simply return 0 here for
            // convenience. In the future, this may need to be changed.
            return 0;
        }
    }

    public static final Motor L_MOTOR = Robot.drivetrain::setLeftMotor;
    public static final Motor R_MOTOR = Robot.drivetrain::setRightMotor;
    public static final DirectionSource GYRO = () -> {
        return Math.toRadians(Robot.drivetrain.getHeading());
    };
    public static final AdvancedPositionSource L_POS_SRC = new EncoderAdvancedPositionSource(
            Robot.drivetrain.getLeftEncoder());
    public static final AdvancedPositionSource R_POS_SRC = new EncoderAdvancedPositionSource(
            Robot.drivetrain.getRightEncoder());
    public static final TimestampSource TIMESTAMP_SOURCE = Timer::getFPGATimestamp;

    public static double kP_l = 0.2, kI_l = 0, kD_l = 0.00015, kV_l = 0.025, kA_l = 0.0015, kDP_l = 0.01;
    public static double kP_h = 0.1, kI_h = 0, kD_h = 0.00025, kV_h = 0.007, kA_h = 0.002, kDP_h = 0.01;
    public static double updateDelay = 0.75;

    // This is the gear the robot must be in for trajectory following
    // If set to null, the robot will accept both
    public static Drivetrain.Gear gearToUse = null;

    public final Followable<TankDriveMoment> profile;
    public Follower<TankDriveMoment> follower;

    /**
     * Constructs a new {@link FollowTrajectory} command.
     * 
     * If the profile provided for following is an instance of
     * {@link DynamicFollowable}, then this Command will automatically use a
     * {@link DynamicTankDriveFollower} instead of a regular
     * {@link TankDriveFollower}.
     * 
     * @param trajectory The trajectory to follow
     */
    public FollowTrajectory(Followable<TankDriveMoment> trajectory) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        this.profile = trajectory;
    }

    private Drivetrain.Gear startingGear;

    // Called just before this Command runs the first time
    // Note we made this method public! This is so that Commands that wrap around
    // this one have an easier time.
    @Override
    public void initialize() {
        RobotLogger.logInfoFine("FollowTrajectory started");
        Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
        // If the gear to use is not null, make sure the robot is in the correct gear
        if (gearToUse != null) {
            startingGear = Robot.drivetrain.getGear();
            Robot.drivetrain.setGear(gearToUse);
        }

        // Check the current gear since high and low gear have different gains
        if (Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH) {
            // Check if the profile can be followed using a dynamic follower instead of a regular one
            if (profile instanceof DynamicFollowable) {
                follower = new DynamicTankDriveFollower((DynamicFollowable<TankDriveMoment>) profile, L_MOTOR, R_MOTOR,
                        L_POS_SRC, R_POS_SRC, TIMESTAMP_SOURCE, GYRO, kV_h, kA_h, kP_h, kI_h, kD_h, kDP_h, updateDelay);
            } else {
                follower = new TankDriveFollower(profile, L_MOTOR, R_MOTOR, L_POS_SRC, R_POS_SRC, TIMESTAMP_SOURCE,
                        GYRO, kV_h, kA_h, kP_h, kI_h, kD_h, kDP_h);
            }
        } else {
            if (profile instanceof DynamicFollowable) {
                follower = new DynamicTankDriveFollower((DynamicFollowable<TankDriveMoment>) profile, L_MOTOR, R_MOTOR,
                        L_POS_SRC, R_POS_SRC, TIMESTAMP_SOURCE, GYRO, kV_l, kA_l, kP_l, kI_l, kD_l, kDP_l, updateDelay);
            } else {
                follower = new TankDriveFollower(profile, L_MOTOR, R_MOTOR, L_POS_SRC, R_POS_SRC, TIMESTAMP_SOURCE,
                        GYRO, kV_l, kA_l, kP_l, kI_l, kD_l, kDP_l);
            }
        }

        follower.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        follower.run();

        if (Robot.isInDebugMode) {
            if (follower instanceof TankDriveFollower) {
                TankDriveFollower f = (TankDriveFollower) follower;
                SmartDashboard.putNumber("Follower Left Output", f.lastLeftOutput());
                SmartDashboard.putNumber("Follower Right Output", f.lastRightOutput());

                TankDriveMoment m = f.lastMoment();

                SmartDashboard.putNumber("Follower Left Velocity", m.getLeftVelocity());
                SmartDashboard.putNumber("Follower Right Velocity", m.getRightVelocity());

                SmartDashboard.putNumber("Follower Left Acceleration", m.getLeftAcceleration());
                SmartDashboard.putNumber("Follower Right Acceleration", m.getRightAcceleration());

                SmartDashboard.putNumber("Follower Left Error", f.lastLeftError());
                SmartDashboard.putNumber("Follower Right Error", f.lastRightError());

                SmartDashboard.putNumber("Follower Left Error Derivative", f.lastLeftDerivative());
                SmartDashboard.putNumber("Follower Right Error Derivative", f.lastRightDerivative());

                SmartDashboard.putNumber("Follower Directional Error", f.lastDirectionalError());
            } else if (follower instanceof DynamicTankDriveFollower) {
                DynamicTankDriveFollower f = (DynamicTankDriveFollower) follower;
                SmartDashboard.putNumber("Follower Left Output", f.lastLeftOutput());
                SmartDashboard.putNumber("Follower Right Output", f.lastRightOutput());

                TankDriveMoment m = f.lastMoment();

                SmartDashboard.putNumber("Follower Left Velocity", m.getLeftVelocity());
                SmartDashboard.putNumber("Follower Right Velocity", m.getRightVelocity());

                SmartDashboard.putNumber("Follower Left Acceleration", m.getLeftAcceleration());
                SmartDashboard.putNumber("Follower Right Acceleration", m.getRightAcceleration());

                SmartDashboard.putNumber("Follower Left Error", f.lastLeftError());
                SmartDashboard.putNumber("Follower Right Error", f.lastRightError());

                SmartDashboard.putNumber("Follower Left Error Derivative", f.lastLeftDerivative());
                SmartDashboard.putNumber("Follower Right Error Derivative", f.lastRightDerivative());

                SmartDashboard.putNumber("Follower Directional Error", f.lastDirectionalError());
            }
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

        if (gearToUse != null) {
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

        if (gearToUse != null) {
            Robot.drivetrain.setGear(startingGear);
        }

        RobotLogger.logInfoFine("FollowTrajectory interrupted");
    }

    /**
     * Retrieves the correct {@link RobotSpecs} based on the gear to use in autos
     * and/or the robot's current gear.
     * 
     * @return The correct {@link RobotSpecs}
     */
    public static RobotSpecs getSpecs() {
        // If the gear to use in autos is specified, generate trajectories based on that
        if (gearToUse != null) {
            return gearToUse == Drivetrain.Gear.HIGH ? RobotMap.specsHigh : RobotMap.specsLow;
        } else {
            // Otherwise generate it based on the robot's current gear
            return Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH ? RobotMap.specsHigh : RobotMap.specsLow;
        }
    }
}
