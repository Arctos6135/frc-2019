/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.Map;
import java.util.TimerTask;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.ShutdownJetson;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.sandstorm.AutoDispatcher;
import frc.robot.misc.AutoPaths;
import frc.robot.misc.BeautifulRobotDriver;
import frc.robot.misc.RobotLogger;
import frc.robot.subsystems.BeautifulRobot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Essie;
import frc.robot.subsystems.Hank;
import frc.robot.subsystems.PressureSensor;
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
    public static Climber climber;
    public static OI oi;
    public static PressureSensor pressureSensor;

    public static Command autoCommand;

    static SendableChooser<AutoDispatcher.Mode> modeChooser = new SendableChooser<>();
    static SendableChooser<AutoDispatcher.HabLevel> habLevelChooser = new SendableChooser<>();
    static SendableChooser<AutoDispatcher.Side> sideChooser = new SendableChooser<>();
    static SendableChooser<AutoDispatcher.RobotSide> robotSideChooser = new SendableChooser<>();
    static SendableChooser<Drivetrain.Gear> followerGearChooser = new SendableChooser<>();
    static SendableChooser<Drivetrain.Gear> matchStartGearChooser = new SendableChooser<>();

    public static boolean isInDebugMode = false;

    public static final String FRONT_CAMERA_URL = "http://10.61.35.19:1180/stream?topic=/main_camera/image_raw&quality=20&width=320&height=180";
    public static final String REAR_CAMERA_URL = "http://10.61.35.19:1180/stream?topic=/secondary_camera/image_raw&quality=20&width=320&height=240";
    private static final NetworkTableEntry cameraURLsEntry = NetworkTableInstance.getDefault()
            .getTable("CameraPublisher").getSubTable("JetsonCameras").getEntry("streams");
    private static String[] cameraStreamURLs = new String[] { FRONT_CAMERA_URL, REAR_CAMERA_URL };

    public static void setMainCameraURL(String url) {
        cameraStreamURLs[0] = url;
        cameraURLsEntry.setStringArray(cameraStreamURLs);
    }

    public static void setSecondaryCameraURL(String url) {
        cameraStreamURLs[1] = url;
        cameraURLsEntry.setStringArray(cameraStreamURLs);
    }

    /**
     * This Shuffleboard tab is used for pre-match configuration, such as autos.
     */
    public static final ShuffleboardTab prematchTab = Shuffleboard.getTab("Pre-match");
    /**
     * This Shuffleboard tab is used for normal operation during driving.
     */
    public static final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    /**
     * This Shuffleboard tab is used for regular debug information.
     */
    public static final ShuffleboardTab debugTab = Shuffleboard.getTab("Debug - General");
    /**
     * This shuffleboard tab is used for vision debug information.
     */
    public static final ShuffleboardTab debugVisionTab = Shuffleboard.getTab("Debug - Vision");
    /**
     * This shuffleboard tab is used for pathfinding/following debug information.
     */
    public static final ShuffleboardTab debugPathfindingTab = Shuffleboard.getTab("Debug - Pathfinding/following");
    /**
     * This Shuffleboard tab is used for miscellaneous options.
     */
    public static final ShuffleboardTab miscTab = Shuffleboard.getTab("Misc");

    // Note: See
    // https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/shuffleboard/Shuffleboard.html
    // for Shuffleboard docs.

    /*************************** Pre-match Tab Entries ***************************/
    public static final NetworkTableEntry validAutoEntry = prematchTab.add("Valid Auto Configuration", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    /*************************** Drive Tab Entries ***************************/

    public static final NetworkTableEntry lastErrorEntry = driveTab.add("Last Error", "")
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry lastWarningEntry = driveTab.add("Last Warning", "")
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    // Note that since commands and subsystems may be uninitialized at this point,
    // default values are used
    public static final NetworkTableEntry visionStatusEntry = driveTab.add("Vision Status", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry driveReversedEntry = driveTab.add("Drive Reversed", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry precisionDriveEntry = driveTab.add("Precision Drive", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry essieCargoEntry = driveTab.add("Essie Cargo", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry pressureLevelEntry = driveTab.add("Pressure Level", 0.0)
            .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 150)).getEntry();
    public static final NetworkTableEntry canClimbEntry = driveTab.add("Can Climb", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry drivetrainGearEntry = driveTab.add("Drivetrain Gear", "LOW")
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    /*************************** Debug Tabs Entries ***************************/

    public static final NetworkTableEntry debugModeEntry = debugTab.add("Debug Mode", isInDebugMode)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry climbingEntry = debugTab.add("Climbing", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry leftDistanceEntry = debugTab.add("Left Distance", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry rightDistanceEntry = debugTab.add("Right Distance", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry leftVelocityEntry = debugTab.add("Left Velocity", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry rightVelocityEntry = debugTab.add("Right Velocity", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry leftAccelerationEntry = debugTab.add("Left Acceleration", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry rightAccelerationEntry = debugTab.add("Right Acceleration", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    public static final NetworkTableEntry followerPHigh = debugPathfindingTab
            .add("Follower kP (High Gear)", FollowTrajectory.kP_h).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerDHigh = debugPathfindingTab
            .add("Follower kD (High Gear)", FollowTrajectory.kD_h).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerVHigh = debugPathfindingTab
            .add("Follower kV (High Gear)", FollowTrajectory.kV_h).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerAHigh = debugPathfindingTab
            .add("Follower kA (High Gear)", FollowTrajectory.kA_h).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerDPHigh = debugPathfindingTab
            .add("Follower kDP (High Gear)", FollowTrajectory.kDP_h).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerPLow = debugPathfindingTab
            .add("Follower kP (Low Gear)", FollowTrajectory.kP_l).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerDLow = debugPathfindingTab
            .add("Follower kD (Low Gear)", FollowTrajectory.kD_l).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerVLow = debugPathfindingTab
            .add("Follower kV (Low Gear)", FollowTrajectory.kV_l).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerALow = debugPathfindingTab
            .add("Follower kA (Low Gear)", FollowTrajectory.kA_l).withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerDPLow = debugPathfindingTab
            .add("Follower kDP (Low Gear)", FollowTrajectory.kDP_l).withWidget(BuiltInWidgets.kTextView).getEntry();
    // No need to configure properties; the default is from -1.0 to 1.0
    public static final NetworkTableEntry followerLeftOutputEntry = debugPathfindingTab.add("Follower Left Output", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar).getEntry();
    public static final NetworkTableEntry followerRightOutputEntry = debugPathfindingTab
            .add("Follower Right Output", 0.0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
    public static final NetworkTableEntry followerLeftErrorEntry = debugPathfindingTab.add("Follower Left Error", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerRightErrorEntry = debugPathfindingTab.add("Follower Right Error", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry followerDirectionalErrorEntry = debugPathfindingTab
            .add("Follower Directional Error", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    public static final NetworkTableEntry visionEnabledEntry = debugVisionTab.add("Vision Enabled", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    public static final NetworkTableEntry visionXOffsetEntry = debugVisionTab.add("Vision X Offset", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry visionYOffsetEntry = debugVisionTab.add("Vision Y Offset", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();
    public static final NetworkTableEntry visionAngleOffsetEntry = debugVisionTab.add("Vision Angle Offset", 0.0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    /*************************** Misc Tab Entries ***************************/

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        hank = new Hank();
        vision = new Vision();
        drivetrain = new Drivetrain();
        essie = new Essie();
        climber = new Climber();
        beautifulRobot = new BeautifulRobot();
        pressureSensor = new PressureSensor();
        oi = new OI();

        // Warm up RobotPathfinder and generate auto paths
        FollowTrajectory.warmupRobotPathfinder(10);
        AutoPaths.generateAll();

        beautifulRobot.init();
        beautifulRobot.setEnabled(true);
        beautifulRobot.setCustomColor((byte) 255, (byte) 102, (byte) 0);
        beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_HIGH, (byte) 0);
        beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_LOW, (byte) 0x80);
        beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.RAINBOW_DASH);
        beautifulRobot.turnOn();

        // Wait for the DS to connect before starting the logger
        // This is important as the roboRIO's system time is only updated when the DS is
        // connected
        while (!DriverStation.getInstance().isDSAttached()) {
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.RAINBOW);
        beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_HIGH, (byte) 0x01);
        beautifulRobot.writeCommand(BeautifulRobotDriver.Operation.SPEED_LOW, (byte) 0x00);

        try {
            RobotLogger.init();
        } catch (IOException e) {
            e.printStackTrace();
            lastErrorEntry.setString("Failed to initialize logger!");
        }
        RobotLogger.logInfo("Logger initialized");
        beautifulRobot.setAlliance(DriverStation.getInstance().getAlliance());

        java.util.Timer timer = new java.util.Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                RobotLogger.logInfoFine("Battery Voltage: " + RobotController.getBatteryVoltage());
            }
        }, 10, 2000);

        setMainCameraURL(FRONT_CAMERA_URL);
        setSecondaryCameraURL(REAR_CAMERA_URL);

        // Add a shutdown Jetson command
        debugTab.add("Shutdown Jetson", new ShutdownJetson()).withWidget(BuiltInWidgets.kCommand);

        // Create auto chooser
        modeChooser.setDefaultOption("None", AutoDispatcher.Mode.NONE);
        modeChooser.addOption("Cargo Ship Front", AutoDispatcher.Mode.FRONT);
        modeChooser.addOption("Cargo Ship Side", AutoDispatcher.Mode.SIDE);
        modeChooser.addOption("Vision", AutoDispatcher.Mode.VISION);
        modeChooser.addOption("Side Vision", AutoDispatcher.Mode.SIDE_VISION);
        modeChooser.addOption("Debug", AutoDispatcher.Mode.DEBUG);
        prematchTab.add("Auto Mode", modeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
        habLevelChooser.setDefaultOption("Level 1", AutoDispatcher.HabLevel.ONE);
        habLevelChooser.addOption("Level 2", AutoDispatcher.HabLevel.TWO);
        prematchTab.add("Auto Start Hab Level", habLevelChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
        sideChooser.setDefaultOption("Left", AutoDispatcher.Side.LEFT);
        sideChooser.addOption("Right", AutoDispatcher.Side.RIGHT);
        prematchTab.add("Auto Side", sideChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
        robotSideChooser.setDefaultOption("Hank Side", AutoDispatcher.RobotSide.HANK);
        robotSideChooser.addOption("Essie Side", AutoDispatcher.RobotSide.ESSIE);
        prematchTab.add("Auto Robot Side", robotSideChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        // Create follower gear chooser and match start gear chooser
        followerGearChooser.setDefaultOption("Low Gear", Drivetrain.Gear.LOW);
        followerGearChooser.addOption("High Gear", Drivetrain.Gear.HIGH);
        followerGearChooser.addOption("All Gears", null);
        debugTab.add("Trajectory Follower Gear", followerGearChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        matchStartGearChooser.setDefaultOption("Low Gear", Drivetrain.Gear.LOW);
        matchStartGearChooser.addOption("High Gear", Drivetrain.Gear.HIGH);
        matchStartGearChooser.addOption("Current Gear", null);
        prematchTab.add("Match Start Gear", matchStartGearChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        RobotLogger.logInfo("Basic initialization complete. Waiting for vision to come online...");

        // Wait for vision to be ready if it's not already
        if (!vision.ready()) {
            long start = System.currentTimeMillis();
            try {
                // Wait for up to a minute for the vision subsystem to come online
                while (!vision.ready() && System.currentTimeMillis() - start < 60000) {
                    Thread.sleep(300);
                    if (OI.operatorController.getRawButton(OI.Controls.SKIP_VISION_INIT)) {
                        break;
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        visionStatusEntry.setBoolean(vision.ready());

        if (!vision.ready()) {
            RobotLogger.logError("Wait for vision initialization timed out");
            OI.errorRumbleDriverMajor.execute();
            OI.errorRumbleOperatorMajor.execute();
        } else {
            try {
                vision.setVisionEnabled(false);
            } catch (VisionException e) {
                RobotLogger.logError("Vision went offline unexpectedly");
            }
        }

        // Put the gyro on the dashboard
        // TODO: Change tab?
        driveTab.add("Gyro", drivetrain.new Gyro()).withWidget(BuiltInWidgets.kGyro);

        RobotLogger.logInfo("Robot initialization complete");
    }

    /**
     * Updates a bunch of tunable values based on new values from Shuffleboard.
     */
    public static void getTuningEntries() {
        Drivetrain.Gear newGearToUse = followerGearChooser.getSelected();
        // Change the gear to use in autos
        // If the option was changed, the auto paths have to be regenerated
        if (FollowTrajectory.gearToUse != newGearToUse) {
            RobotLogger.logInfoFine(
                    "Auto gear has been changed to " + newGearToUse.toString() + ". Regenerating trajectories...");
            FollowTrajectory.gearToUse = newGearToUse;
            AutoPaths.generateAll();
        }

        FollowTrajectory.kP_h = followerPHigh.getDouble(FollowTrajectory.kP_h);
        FollowTrajectory.kD_h = followerDHigh.getDouble(FollowTrajectory.kD_h);
        FollowTrajectory.kV_h = followerVHigh.getDouble(FollowTrajectory.kV_h);
        FollowTrajectory.kA_h = followerAHigh.getDouble(FollowTrajectory.kA_h);
        FollowTrajectory.kDP_h = followerDPHigh.getDouble(FollowTrajectory.kDP_h);

        FollowTrajectory.kP_l = followerPLow.getDouble(FollowTrajectory.kP_l);
        FollowTrajectory.kD_l = followerDLow.getDouble(FollowTrajectory.kD_l);
        FollowTrajectory.kV_l = followerVLow.getDouble(FollowTrajectory.kV_l);
        FollowTrajectory.kA_l = followerALow.getDouble(FollowTrajectory.kA_l);
        FollowTrajectory.kDP_l = followerDPLow.getDouble(FollowTrajectory.kDP_l);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Vision status is outputted regardless of current state
        visionStatusEntry.setBoolean(vision.ready());
        driveReversedEntry.setBoolean(TeleopDrive.isReversed());
        precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive());
        essieCargoEntry.setBoolean(essie.hasCargo());
        pressureLevelEntry.setDouble(pressureSensor.getPressure());
        canClimbEntry.setBoolean(pressureSensor.canClimb());

        if (isInDebugMode) {
            drivetrainGearEntry.setString(drivetrain.getGear() == Drivetrain.Gear.HIGH ? "HIGH" : "LOW");
            leftDistanceEntry.setDouble(drivetrain.getLeftDistance());
            rightDistanceEntry.setDouble(drivetrain.getRightDistance());
            leftVelocityEntry.setDouble(drivetrain.getLeftSpeed());
            rightVelocityEntry.setDouble(drivetrain.getRightSpeed());
            var accelerations = drivetrain.getAccelerations();
            leftAccelerationEntry.setDouble(accelerations[0]);
            rightAccelerationEntry.setDouble(accelerations[1]);

            visionEnabledEntry.setBoolean(vision.getVisionEnabled());
            if (Robot.vision.getVisionEnabled()) {
                try {
                    visionXOffsetEntry.setDouble(vision.getTargetXOffset());
                    visionYOffsetEntry.setDouble(vision.getTargetYOffset());
                    visionAngleOffsetEntry.setDouble(vision.getTargetAngleOffset());
                } catch (VisionException e) {
                    RobotLogger.logError("Vision went offline unexpectedly");
                }
            }
        }
    }

    /**
     * Note: Although the 2019 game technically has no autonomous period as it is
     * replaced by the Sandstorm, to keep names consistent, it is still referred to
     * as the "autonomous period". This means that although the robot can still
     * receive operator control, the methods for autonomous mode are called at the
     * start of the game instead of those for teleop.
     */
    @Override
    public void autonomousInit() {
        RobotLogger.logInfo("Autonomous mode enabled");
        if (beautifulRobot.getColor() != BeautifulRobotDriver.Color
                .fromAlliance(DriverStation.getInstance().getAlliance())) {
            // If the alliance colour is not set, do it here
            beautifulRobot.setColor(BeautifulRobotDriver.Color.fromAlliance(DriverStation.getInstance().getAlliance()));
            RobotLogger
                    .logInfoFine("BeautifulRobot alliance colour changed to " + beautifulRobot.getColor().toString());
        }
        // Set the initial gear
        Drivetrain.Gear matchStartGear = matchStartGearChooser.getSelected();
        if (matchStartGear != null) {
            RobotLogger.logInfoFine("Match start gear is " + matchStartGear.toString());
            Robot.drivetrain.setGear(matchStartGear);
        }
        // Un-reverse driving
        TeleopDrive.setReversed(false);

        beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.PULSATING);

        autoCommand = AutoDispatcher.getAuto(modeChooser.getSelected(), habLevelChooser.getSelected(),
                sideChooser.getSelected(), robotSideChooser.getSelected());
        if (autoCommand != null) {
            autoCommand.start();
            RobotLogger.logInfo("Autonomous command started: " + autoCommand.getClass().getName());
        } else {
            RobotLogger.logWarning("No auto exists for the specified configuration");
            OI.errorRumbleDriverMinor.execute();
            OI.errorRumbleOperatorMinor.execute();
        }
    }

    /**
     * Note: Although the 2019 game technically has no autonomous period as it is
     * replaced by the Sandstorm, to keep names consistent, it is still referred to
     * as the "autonomous period". This means that although the robot can still
     * receive operator control, the methods for autonomous mode are called at the
     * start of the game instead of those for teleop.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        RobotLogger.logInfo("Teleop mode enabled");
        if (beautifulRobot.getColor() != BeautifulRobotDriver.Color
                .fromAlliance(DriverStation.getInstance().getAlliance())) {
            // If the alliance colour is not set, do it here
            beautifulRobot.setColor(BeautifulRobotDriver.Color.fromAlliance(DriverStation.getInstance().getAlliance()));
            RobotLogger
                    .logInfoFine("BeautifulRobot alliance colour changed to " + beautifulRobot.getColor().toString());
        }
        beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.MOVING_PULSE);
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
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit() {
        RobotLogger.logInfo("Robot disabled");
        // Flush the log buffer when the robot is disabled
        RobotLogger.flush();
        beautifulRobot.setPattern(BeautifulRobotDriver.Pattern.RAINBOW);
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();

        // Check if the auto configuration is valid
        validAutoEntry.setBoolean(AutoDispatcher.getAuto(modeChooser.getSelected(), habLevelChooser.getSelected(),
                sideChooser.getSelected(), robotSideChooser.getSelected()) != null);
        if (isInDebugMode) {
            getTuningEntries();
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
