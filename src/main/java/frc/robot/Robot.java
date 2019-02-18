/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.ShutdownJetson;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.sandstorm.AutoDispatcher;
import frc.robot.misc.AutoPaths;
import frc.robot.misc.powermanagement.PowerManager;
import frc.robot.subsystems.BeautifulRobot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Essie;
import frc.robot.subsystems.Hank;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Hank hank;
    public static Drivetrain drivetrain;
    public static Essie essie;
    public static Vision vision;
    public static BeautifulRobot beautifulRobot;
    public static OI oi;
    
    public static Command autoCommand;

    static SendableChooser<AutoDispatcher.Mode> modeChooser = new SendableChooser<>();
    static SendableChooser<AutoDispatcher.HabLevel> habLevelChooser = new SendableChooser<>();
    static SendableChooser<Drivetrain.Gear> followerGearChooser = new SendableChooser<>();
    static SendableChooser<Drivetrain.Gear> matchStartGearChooser = new SendableChooser<>();

    public static boolean isInDebugMode = false;

    public static final String FRONT_CAMERA_URL = "http://10.61.35.19:1180/stream?topic=/main_camera/image_raw&quality=30&width=640&height=360";
    public static final String REAR_CAMERA_URL = "http://10.61.35.19:1180/stream?topic=/secondary_camera/image_raw&quality=30";
    public static final NetworkTableEntry mainCameraUrl = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("main-stream-url");
    public static final NetworkTableEntry secondaryCameraUrl = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("secondary-stream-url");

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        hank = new Hank();
        vision = new Vision();
        drivetrain = new Drivetrain();
        essie = new Essie();
        beautifulRobot = new BeautifulRobot();
        oi = new OI();
        // Clear the last error
        SmartDashboard.putString("Last Error", "");

        mainCameraUrl.setString(FRONT_CAMERA_URL);
        secondaryCameraUrl.setString(REAR_CAMERA_URL);

        AutoPaths.generateAll();

        PowerManager.startCompressorPowerManagement(11.0);
        PowerManager.startDrivetrainPowerManagement(8.5, 0.4);
        PowerManager.startEssiePowerManagement();

        beautifulRobot.init();
        beautifulRobot.setEnabled(true);
        beautifulRobot.setAlliance(DriverStation.getInstance().getAlliance());
        beautifulRobot.turnOn();

        SmartDashboard.putData("Shutdown Jetson", new ShutdownJetson());

        // Create auto chooser
        modeChooser.setDefaultOption("None", AutoDispatcher.Mode.NONE);
        modeChooser.addOption("Left Side", AutoDispatcher.Mode.LEFT);
        modeChooser.addOption("Right Side", AutoDispatcher.Mode.RIGHT);
        modeChooser.addOption("Aligned", AutoDispatcher.Mode.ALIGNED);
        modeChooser.addOption("Vision", AutoDispatcher.Mode.VISION);
        modeChooser.addOption("Debug", AutoDispatcher.Mode.DEBUG);
        habLevelChooser.setDefaultOption("Level 1", AutoDispatcher.HabLevel.ONE);
        habLevelChooser.addOption("Level 2", AutoDispatcher.HabLevel.TWO);
        
        followerGearChooser.setDefaultOption("Low Gear", Drivetrain.Gear.LOW);
        followerGearChooser.addOption("High Gear", Drivetrain.Gear.HIGH);
        followerGearChooser.addOption("All Gears", null);

        matchStartGearChooser.setDefaultOption("Low Gear", Drivetrain.Gear.LOW);
        matchStartGearChooser.addOption("High Gear", Drivetrain.Gear.HIGH);
        matchStartGearChooser.addOption("Current Gear", null);

        SmartDashboard.putData("Auto Mode", modeChooser);
        SmartDashboard.putData("Starting Hab Level", habLevelChooser);
        SmartDashboard.putData("Match Start Gear", matchStartGearChooser);
        
        // Warm up RobotPathfinder
        SmartDashboard.putNumber("Final Generation Time", FollowTrajectory.warmupRobotPathfinder(10));
        
        // Wait for vision to be ready if it's not already
        SmartDashboard.putBoolean("Vision Status", false);
        if(!vision.ready()) {
            long start = System.currentTimeMillis();
            try {
                // Wait for up to a minute for the vision subsystem to come online
                while(!vision.ready() && System.currentTimeMillis() - start < 60000) {
                    Thread.sleep(300);
                    if(OI.operatorController.getRawButton(OI.Controls.SKIP_VISION_INIT)) {
                        break;
                    }
                }
            }
            catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
        SmartDashboard.putBoolean("Vision Status", vision.ready());

        if(!vision.ready()) {
            Robot.error("Wait for vision initialization timed out");
            OI.errorRumbleDriverMajor.execute();
            OI.errorRumbleOperatorMajor.execute();
        }
        else {
            try {
                vision.setVisionEnabled(false);
            }
            catch(VisionException e) {
                Robot.error("Vision went offline unexpectedly");
            }
        }

        if(isInDebugMode) {
            putTuningEntries();
        }
    }

    /**
     * Puts a bunch of tunable values to SmartDashboard for tuning.
     */
    public static void putTuningEntries() {
        SmartDashboard.putData("Path Follower Gear", followerGearChooser);

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
    public static void getTuningEntries() {
        Drivetrain.Gear newGearToUse = followerGearChooser.getSelected();
        // Change the gear to use in autos
        // If the option was changed, the auto paths have to be regenerated
        if(FollowTrajectory.gearToUse != newGearToUse) {
            FollowTrajectory.gearToUse = newGearToUse;
            AutoPaths.generateAll();
        }

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
        SmartDashboard.putBoolean("Drive Reversed", TeleopDrive.isReversed());
        SmartDashboard.putBoolean("Essie Cargo", essie.hasCargo());
        SmartDashboard.putBoolean("Precision Drive", TeleopDrive.isPrecisionDrive());
        SmartDashboard.putBoolean("Debug", isInDebugMode);
        
        SmartDashboard.putBoolean("Essie Cargo", essie.hasCargo());
        if(isInDebugMode) {
            SmartDashboard.putNumber("Essie High", RobotMap.essieMotorHighUnprotected.get());
            SmartDashboard.putNumber("Essie Low", RobotMap.essieMotorLowUnprotected.get());        
            SmartDashboard.putNumber("Gyro Reading", drivetrain.getHeading());

            SmartDashboard.putString("Drivetrain Gear", drivetrain.getGear() == Drivetrain.Gear.HIGH ? "HIGH" : "LOW");
            SmartDashboard.putNumber("Left Distance", drivetrain.getLeftDistance());
            SmartDashboard.putNumber("Right Distance", drivetrain.getRightDistance());
            SmartDashboard.putNumber("Left Velocity", drivetrain.getLeftSpeed());
            SmartDashboard.putNumber("Right Velocity", drivetrain.getRightSpeed());
            var accelerations = drivetrain.getAccelerations();
            SmartDashboard.putNumber("Left Acceleration", accelerations[0]);
            SmartDashboard.putNumber("Right Acceleration", accelerations[1]);

            SmartDashboard.putBoolean("Vision Enabled", vision.getVisionEnabled());
            if(Robot.vision.getVisionEnabled()) {
                try {
                    SmartDashboard.putNumber("X Offset", vision.getTargetXOffset());
                    SmartDashboard.putNumber("Y Offset", vision.getTargetYOffset());
                    SmartDashboard.putNumber("Angle Offset", vision.getTargetAngleOffset());
                }
                catch(VisionException e) {
                    Robot.error("Vision went offline unexpectedly");
                }
            }
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
        if(beautifulRobot.getColor() != BeautifulRobot.Color.RED && beautifulRobot.getColor() != BeautifulRobot.Color.BLUE) {
            // If the alliance colour is not set, do it here
            beautifulRobot.setColor(BeautifulRobot.Color.fromAlliance(DriverStation.getInstance().getAlliance()));
        }
        // Set the initial gear
        Drivetrain.Gear matchStartGear = matchStartGearChooser.getSelected();
        if(matchStartGear != null) {
            Robot.drivetrain.setGear(matchStartGear);
        }
        // Un-reverse driving
        TeleopDrive.setReversed(false);

        beautifulRobot.setPattern(BeautifulRobot.Pattern.PULSATING);
        AutoDispatcher.Mode mode = modeChooser.getSelected();
        AutoDispatcher.HabLevel level = habLevelChooser.getSelected();

        try {
            autoCommand = AutoDispatcher.getAuto(level, mode);
            autoCommand.start();
        }
        catch(AutoDispatcher.AutoNotFoundException e) {
            Robot.error("No auto exists for the specified configuration");
            OI.errorRumbleDriverMinor.execute();
            OI.errorRumbleDriverMajor.execute();
            autoCommand = null;
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
        if(beautifulRobot.getColor() != BeautifulRobot.Color.RED && beautifulRobot.getColor() != BeautifulRobot.Color.BLUE) {
            // If the alliance colour is not set, do it here
            beautifulRobot.setColor(BeautifulRobot.Color.fromAlliance(DriverStation.getInstance().getAlliance()));
        }
        beautifulRobot.setPattern(BeautifulRobot.Pattern.MOVING_PULSE);
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
        beautifulRobot.setPattern(BeautifulRobot.Pattern.RAINBOW);
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();

        // Check if the auto configuration is valid
        try {
            AutoDispatcher.getAuto(habLevelChooser.getSelected(), modeChooser.getSelected());
            SmartDashboard.putBoolean("Valid Auto Configuration", true);
        }
        catch(AutoDispatcher.AutoNotFoundException e) {
            SmartDashboard.putBoolean("Valid Auto Configuration", false);
        }
        if(isInDebugMode) {
            getTuningEntries();
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public static void error(String error) {
        SmartDashboard.putString("Last Error", error);
        DriverStation.reportError(error, true);
    }
    public static void warning(String warning) {
        SmartDashboard.putString("Last Warning", warning);
        DriverStation.reportWarning(warning, true);
    }
}
