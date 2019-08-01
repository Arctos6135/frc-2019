/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.misc.RobotLogger;

public class DriveDistance extends Command {

    final double distance;

    Followable<TankDriveMoment> profile;
    FollowTrajectory followerCommand;

    public DriveDistance(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);

        this.distance = distance;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        RobotLogger.logInfoFiner("Driving distance " + distance);
        profile = new TrapezoidalTankDriveProfile(FollowTrajectory.getSpecs(), distance);
        // Wrap around a FollowTrajectory
        followerCommand = new FollowTrajectory(profile);
        followerCommand.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        followerCommand.execute();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return followerCommand.isFinished();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        followerCommand.end();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        followerCommand.interrupted();
    }
}
