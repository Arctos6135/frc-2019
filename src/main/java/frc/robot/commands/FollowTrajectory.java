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
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveGains;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveRobot;
import com.arctos6135.stdplug.api.datatypes.PIDVADPGains;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
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

    private static final Motor L_MOTOR = Robot.drivetrain::setLeftMotor;
    private static final Motor R_MOTOR = Robot.drivetrain::setRightMotor;
    private static final DirectionSource GYRO = () -> {
        return Math.toRadians(Robot.drivetrain.getHeading());
    };
    private static final AdvancedPositionSource L_POS_SRC = new EncoderAdvancedPositionSource(
            Robot.drivetrain.getLeftEncoder());
    private static final AdvancedPositionSource R_POS_SRC = new EncoderAdvancedPositionSource(
            Robot.drivetrain.getRightEncoder());
    private static final TimestampSource TIMESTAMP_SOURCE = Timer::getFPGATimestamp;

    private static final TankDriveRobot ROBOT_COMPONENTS = new TankDriveRobot(L_MOTOR, R_MOTOR, L_POS_SRC, R_POS_SRC,
            TIMESTAMP_SOURCE, GYRO);

    // These fields are made public so they can be configured from the dashboard
    public static final PIDVADPGains GAINS_L = new PIDVADPGains(0.2, 0.0, 0.00015, 0.025, 0.0015, 0.01);
    public static final PIDVADPGains GAINS_H = new PIDVADPGains(0.1, 0.0, 0.00025, 0.007, 0.002, 0.01);
    public static double updateDelay = 0.75;

    // This is the gear the robot must be in for trajectory following
    // If set to null, the robot will accept both
    public static Drivetrain.Gear gearToUse = null;

    private final Followable<TankDriveMoment> profile;
    private Follower<TankDriveMoment> follower;

    /**
     * Constructs a {@link TankDriveGains} object from a {@link PIDVADPGains}
     * object.
     * 
     * This is because the class has to store the gains as StdPlug
     * {@link PIDVADPGains} objects in order to have them sent to Shuffleboard and
     * configured, but the {@link Follower} constructors only take
     * {@link TankDriveGains}.
     * 
     * @param stdPlugGains The {@link PIDVADPGains} object
     */
    private static TankDriveGains constructGainsObject(PIDVADPGains stdPlugGains) {
        return new TankDriveGains(stdPlugGains.getV(), stdPlugGains.getA(), stdPlugGains.getP(), stdPlugGains.getI(),
                stdPlugGains.getD(), stdPlugGains.getDP());
    }

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
            // Check if the profile can be followed using a dynamic follower instead of a
            // regular one
            if (profile instanceof DynamicFollowable) {
                follower = new DynamicTankDriveFollower((DynamicFollowable<TankDriveMoment>) profile, ROBOT_COMPONENTS,
                        constructGainsObject(GAINS_H), updateDelay);
            } else {
                follower = new TankDriveFollower(profile, ROBOT_COMPONENTS, constructGainsObject(GAINS_H));
            }
        } else {
            if (profile instanceof DynamicFollowable) {
                follower = new DynamicTankDriveFollower((DynamicFollowable<TankDriveMoment>) profile, ROBOT_COMPONENTS,
                        constructGainsObject(GAINS_L), updateDelay);
            } else {
                follower = new TankDriveFollower(profile, ROBOT_COMPONENTS, constructGainsObject(GAINS_L));
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

                Robot.followerLeftOutputEntry.setDouble(f.lastLeftOutput());
                Robot.followerRightOutputEntry.setDouble(f.lastRightOutput());
                Robot.followerLeftErrorEntry.setDouble(f.lastLeftError());
                Robot.followerRightErrorEntry.setDouble(f.lastRightError());
                Robot.followerDirectionalErrorEntry.setDouble(f.lastDirectionalError());
            } else if (follower instanceof DynamicTankDriveFollower) {
                DynamicTankDriveFollower f = (DynamicTankDriveFollower) follower;

                Robot.followerLeftOutputEntry.setDouble(f.lastLeftOutput());
                Robot.followerRightOutputEntry.setDouble(f.lastRightOutput());
                Robot.followerLeftErrorEntry.setDouble(f.lastLeftError());
                Robot.followerRightErrorEntry.setDouble(f.lastRightError());
                Robot.followerDirectionalErrorEntry.setDouble(f.lastDirectionalError());
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
