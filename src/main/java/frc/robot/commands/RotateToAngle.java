/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveRotationProfile;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Rotates the robot in place a certain number of degrees in a specified direction.
 * This command uses the {@link frc.robot.commands.FollowTrajectory FollowTrajectory} command internally
 * and thus requires RobotPathfinder to be properly tuned.
 */
public class RotateToAngle extends CommandBase {

    /**
     * The direction to turn in.
     */
    public enum Direction {
        LEFT, RIGHT;
    }

    Followable<TankDriveMoment> profile;
    FollowTrajectory followerCommand;

    double angle;
    Direction direction;

    /**
     * Creates a new rotate to angle command.
     * @param angle The angle, in degrees, to turn
     * @param direction The direction to turn in
     */
    public RotateToAngle(double angle, Direction direction) {
        addRequirements(Robot.drivetrain);
        this.angle = angle;
        this.direction = direction;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Robot.logger.logInfoFiner("Generating trajectory to rotate to angle...");
        // Use a RobotPathfinder trajectory here to save time and improve accuracy with the already tuned PIDs
        profile = new TrapezoidalTankDriveRotationProfile(FollowTrajectory.getSpecs(), 
                direction == Direction.LEFT ? Math.toRadians(angle) : Math.toRadians(-angle));
        followerCommand = new FollowTrajectory(profile);
        Robot.logger.logInfoFiner("Rotating to angle " + angle + " to " + direction.toString());
        // We cannot actually start the FollowTrajectory command, as it also requires drivetrain and will interrupt this command.
        // Therefore we must call its methods manually without handing control to WPILib.
        followerCommand.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        followerCommand.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return followerCommand.isFinished();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        followerCommand.end(interrupted);
    }
}
