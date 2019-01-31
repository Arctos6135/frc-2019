/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.ShutdownJetson;
import frc.robot.subsystems.BeautifulRobot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Drivetrain drivetrain;
    public static Vision vision;
    public static BeautifulRobot beautifulRobot;
    public static OI oi;

    Command autoCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static boolean isInDebugMode = false;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        vision = new Vision("Vision");
        drivetrain = new Drivetrain();
        beautifulRobot = new BeautifulRobot();
        oi = new OI();

        beautifulRobot.init();
        beautifulRobot.setEnabled(true);
        beautifulRobot.setColor(DriverStation.getInstance().getAlliance());
        beautifulRobot.turnOn();

        // chooser.setDefaultOption("Default Auto", new ExampleCommand());
        // chooser.addOption("My Auto", new MyAutoCommand());
        // SmartDashboard.putData("Auto mode", m_chooser);

        SmartDashboard.putData("Shutdown Jetson", new ShutdownJetson());
        
        // Wait for vision to be ready if it's not already
        SmartDashboard.putBoolean("Vision Status", false);
        if(!vision.ready()) {
            vision.notifyWhenReady(this);
            try {
                synchronized(this) {
                    // Wait for a maximum of 1 minute before timing out
                    wait(60000);
                }
            }
            catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
        SmartDashboard.putBoolean("Vision Status", vision.ready());

        if(!vision.ready()) {
            SmartDashboard.putString("Last Error", "Error: Wait for vision initialization timed out.");
            OI.errorRumbleDriverMajor.execute();
            OI.errorRumbleOperatorMajor.execute();
        }
        
        chooser.setDefaultOption("None", null);

        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
            new Waypoint(0.0, 0.0, Math.PI / 2),
            new Waypoint(60.0, 120.0, Math.PI / 2),
        };
        params.alpha = 150.0;
        params.segmentCount = 500;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        TankDriveTrajectory trajectory = new TankDriveTrajectory(RobotMap.specs, params);

        chooser.addOption("Debug Auto", new FollowTrajectory(trajectory));
        SmartDashboard.putData("Auto Mode", chooser);

        if(isInDebugMode) {
            putTuningEntries();
        }
    }

    /**
     * Puts a bunch of tunable values to SmartDashboard for tuning.
     */
    public void putTuningEntries() {
        SmartDashboard.putNumber("Follower kP (High Gear)", FollowTrajectory.kP_h);
        SmartDashboard.putNumber("Follower kD (High Gear)", FollowTrajectory.kD_h);
        SmartDashboard.putNumber("Follower kV (High Gear)", FollowTrajectory.kV_h);
        SmartDashboard.putNumber("Follower kA (High Gear)", FollowTrajectory.kA_h);
        SmartDashboard.putNumber("Follower kDP (High Gear)", FollowTrajectory.kDP_h);

        SmartDashboard.putNumber("Follower kP (Low Gear)", FollowTrajectory.kP_l);
        SmartDashboard.putNumber("Follower kD (Low Gear)", FollowTrajectory.kD_l);
        SmartDashboard.putNumber("Follower kV (Low Gear)", FollowTrajectory.kV_l);
        SmartDashboard.putNumber("Follower kA (Low Gear)", FollowTrajectory.kA_l);
        SmartDashboard.putNumber("Follower kDP (Low Gear)", FollowTrajectory.kDP_l);
    }
    /**
     * Updates a bunch of tunable values based on new values from SmartDashboard.
     */
    public void getTuningEntries() {
        FollowTrajectory.kP_h = SmartDashboard.getNumber("Follower kP (High Gear)", FollowTrajectory.kP_h);
        FollowTrajectory.kD_h = SmartDashboard.getNumber("Follower kD (High Gear)", FollowTrajectory.kD_h);
        FollowTrajectory.kV_h = SmartDashboard.getNumber("Follower kV (High Gear)", FollowTrajectory.kV_h);
        FollowTrajectory.kA_h = SmartDashboard.getNumber("Follower kA (High Gear)", FollowTrajectory.kA_h);
        FollowTrajectory.kDP_h = SmartDashboard.getNumber("Follower kDP (High Gear)", FollowTrajectory.kDP_h);

        FollowTrajectory.kP_l = SmartDashboard.getNumber("Follower kP (Low Gear)", FollowTrajectory.kP_l);
        FollowTrajectory.kD_l = SmartDashboard.getNumber("Follower kD (Low Gear)", FollowTrajectory.kD_l);
        FollowTrajectory.kV_l = SmartDashboard.getNumber("Follower kV (Low Gear)", FollowTrajectory.kV_l);
        FollowTrajectory.kA_l = SmartDashboard.getNumber("Follower kA (Low Gear)", FollowTrajectory.kA_l);
        FollowTrajectory.kDP_l = SmartDashboard.getNumber("Follower kDP (Low Gear)", FollowTrajectory.kDP_l);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Vision status is outputted regardless of current state
        SmartDashboard.putBoolean("Vision Status", vision.ready());
        
        if(isInDebugMode) {
            SmartDashboard.putBoolean("VisiSight", RobotMap.visisight.isBlocked());
            SmartDashboard.putNumber("Gyro Reading", drivetrain.getHeading());

            SmartDashboard.putString("Drivetrain Gear", drivetrain.getGear() == Drivetrain.Gear.HIGH ? "HIGH" : "LOW");
            SmartDashboard.putNumber("Left Distance", drivetrain.getLeftDistance());
            SmartDashboard.putNumber("Right Distance", drivetrain.getRightDistance());
            SmartDashboard.putNumber("Left Velocity", drivetrain.getLeftSpeed());
            SmartDashboard.putNumber("Right Velocity", drivetrain.getRightSpeed());
            var accelerations = drivetrain.getAccelerations();
            SmartDashboard.putNumber("Left Acceleration", accelerations[0]);
            SmartDashboard.putNumber("Right Acceleration", accelerations[1]);
        }
    }

    /**
     * Note: Although the 2019 game technically has no autonomous period
     * as it is replaced by the Sandstorm, to keep names consistent, it
     * is still referred to as the "autonomous period". This means that
     * although the robot can still receive operator control, the methods
     * for autonomous mode are called at the start of the game instead of 
     * those for teleop.
     */
    @Override
    public void autonomousInit() {
        beautifulRobot.setPattern(BeautifulRobot.Pattern.PULSATING);
        autoCommand = chooser.getSelected();

        // schedule the autonomous command (example)
        if (autoCommand != null) {
            autoCommand.start();
        }
    }

    /**
    * Note: Although the 2019 game technically has no autonomous period
    * as it is replaced by the Sandstorm, to keep names consistent, it
    * is still referred to as the "autonomous period". This means that
    * although the robot can still receive operator control, the methods
    * for autonomous mode are called at the start of the game instead of 
    * those for teleop.
    */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        beautifulRobot.setPattern(BeautifulRobot.Pattern.RAINBOW);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        beautifulRobot.setPattern(BeautifulRobot.Pattern.SOLID);
        if(isInDebugMode) {
            getTuningEntries();
            putTuningEntries();
        }
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
