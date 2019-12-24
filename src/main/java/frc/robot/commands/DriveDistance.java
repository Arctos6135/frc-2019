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

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveDistance extends CommandBase {

    final double distance;

    Followable<TankDriveMoment> profile;
    FollowTrajectory followerCommand;

    public DriveDistance(double distance) {
        addRequirements(Robot.drivetrain);

        this.distance = distance;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        Robot.logger.logInfoFiner("Driving distance " + distance);
        profile = new TrapezoidalTankDriveProfile(FollowTrajectory.getSpecs(), distance);
        // Wrap around a FollowTrajectory
        followerCommand = new FollowTrajectory(profile);
        followerCommand.initialize();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        try {
            followerCommand.execute();
        }
        catch(Exception e) {
            System.out.println("DriveDistance exception:");
            e.printStackTrace();
        }
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
