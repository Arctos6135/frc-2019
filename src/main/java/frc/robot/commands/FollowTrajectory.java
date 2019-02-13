/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.follower.Follower.DirectionSource;
import robot.pathfinder.follower.Follower.DistanceSource;
import robot.pathfinder.follower.Follower.Motor;
import robot.pathfinder.follower.Follower.TimestampSource;
import robot.pathfinder.follower.TankFollower;

public class FollowTrajectory extends Command {

    public static final Motor L_MOTOR = Robot.drivetrain::setLeftMotor;
    public static final Motor R_MOTOR = Robot.drivetrain::setRightMotor;
    public static final DirectionSource GYRO = () -> {
        return Math.toRadians(Robot.drivetrain.getHeading());
    };
    public static final DistanceSource L_DISTANCE_SOURCE = Robot.drivetrain::getLeftDistance;
    public static final DistanceSource R_DISTANCE_SOURCE = Robot.drivetrain::getRightDistance;
    public static final TimestampSource TIMESTAMP_SOURCE = Timer::getFPGATimestamp;

    public static double kP_l = 0.025, kD_l = 0.0005, kV_l = 0.025, kA_l = 0.0025, kDP_l = 0;
    public static double kP_h = 0, kD_h = 0, kV_h = 0, kA_h = 0, kDP_h = 0;

    private final TankDriveTrajectory trajectory;
    private TankFollower follower;

    public FollowTrajectory(TankDriveTrajectory trajectory) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        this.trajectory = trajectory;
    }

    // Called just before this Command runs the first time
    // Note we made this method public! This is so that Commands that wrap around this one have an easier time.
    @Override
    public void initialize() {
        if(Robot.drivetrain.getGear() == Drivetrain.Gear.HIGH) {
            follower = new TankFollower(trajectory, L_MOTOR, R_MOTOR, L_DISTANCE_SOURCE, R_DISTANCE_SOURCE, TIMESTAMP_SOURCE, 
                    GYRO, kV_h, kA_h, kP_h, kD_h, kDP_h);
        }
        else {
            follower = new TankFollower(trajectory, L_MOTOR, R_MOTOR, L_DISTANCE_SOURCE, R_DISTANCE_SOURCE, TIMESTAMP_SOURCE, 
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    public void interrupted() {
        follower.stop();
        Robot.drivetrain.setMotors(0, 0);
    }

    /**
     * This static method makes RobotPathfinder generate a bunch of random trajectories in an attempt to
     * activate the JIT and improve performance. 
     * @param count The number of random trajectories to generate
     * @return The final time it takes to generate a trajectory
     */
    public static long warmupRobotPathfinder(int count) {
        for(int i = 0; i < count; i ++) {
            long start = System.currentTimeMillis();
            TrajectoryParams params = new TrajectoryParams();
            params.alpha = Math.random() * 200;
            params.isTank = true;
            params.pathType = PathType.QUINTIC_HERMITE;
            params.segmentCount = 500;
            params.waypoints = new Waypoint[] {
                new Waypoint(0, 0, Math.PI / 2),
                new Waypoint(Math.random() * 100, Math.random() * 100, Math.PI * 2 * Math.random()),
            };
            @SuppressWarnings("unused")
            TankDriveTrajectory trajectory = new TankDriveTrajectory(RobotMap.specs, params);
            if(i == count - 1) {
                return System.currentTimeMillis() - start;
            }
        }
        System.gc();
        return -1;
    }
}
