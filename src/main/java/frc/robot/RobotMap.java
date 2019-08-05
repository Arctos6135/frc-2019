/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.misc.BeautifulRobotDriver;
import frc.robot.misc.RobotLogger;
import frc.robot.misc.protectedmotor.ProtectedMotor;
import robot.pathfinder.core.RobotSpecs;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;

    public static final PowerDistributionPanel pdp = new PowerDistributionPanel();
    public static final Compressor compressor = new Compressor();

    // Encoder constants
	public static final int WHEEL_DIAMETER = 6; //INCHES
	public static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * Math.PI;
    public static final double DRIVE_ENCODER_PPR = 256;
    // 5 / 48 is the gear ratio
    public static final double DISTANCE_PER_PULSE = WHEEL_CIRCUMFRENCE / DRIVE_ENCODER_PPR * 5 / 48;

    public static final DoubleSolenoid hankSolenoid = new DoubleSolenoid(2, 3);
    public static final DoubleSolenoid gearShifter = new DoubleSolenoid(0, 1);
    public static final Solenoid essieClimber = new Solenoid(4);
    public static final Solenoid hankClimber = new Solenoid(5);

    public static final AnalogInput pressureSensor = new AnalogInput(0);

    public static final DigitalInput essieDownMRS = new DigitalInput(6);
    public static final DigitalInput hankDownMRS = new DigitalInput(5);
    public static final DigitalInput essieUpMRS = new DigitalInput(8);
    public static final DigitalInput hankUpMRS = new DigitalInput(7);

    // Drive motors
    public static final WPI_VictorSPX rVictor = new WPI_VictorSPX(0);
    public static final WPI_VictorSPX lVictor = new WPI_VictorSPX(3);
    public static final WPI_TalonSRX lTalon1 = new WPI_TalonSRX(4);
	public static final WPI_TalonSRX lTalon2 = new WPI_TalonSRX(5);
    public static final WPI_TalonSRX rTalon1 = new WPI_TalonSRX(1);
    public static final WPI_TalonSRX rTalon2 = new WPI_TalonSRX(2);

    // Essie motors
    public static final WPI_VictorSPX essieMotorLowUnprotected = new WPI_VictorSPX(7);
    public static final VictorSP essieMotorHighUnprotected = new VictorSP(0);
    public static final ProtectedMotor essieMotorLow = new ProtectedMotor((speed) -> {
        essieMotorLowUnprotected.set(ControlMode.PercentOutput, speed);
    }, 6, 35, 2, () -> {
        OI.errorRumbleOperatorMajor.execute();
        RobotLogger.logError("Critical error: Essie low motor protection tripped");
    });
    public static final ProtectedMotor essieMotorHigh = new ProtectedMotor(essieMotorHighUnprotected::set, 7, 35, 2, 
    () -> {
        OI.errorRumbleOperatorMajor.execute();
        RobotLogger.logError("Critical error: Essie high motor protection tripped");
    });
    public static final DigitalInput essiePhotoElectric = new DigitalInput(4);

    // navX
    public static final AHRS ahrs = new AHRS(I2C.Port.kOnboard);
	public static Encoder rightEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	public static Encoder leftEncoder = new Encoder(2, 3, true, EncodingType.k4X);
	
    public static final BeautifulRobotDriver beautifulRobotDriver = new BeautifulRobotDriver(Port.kMXP);
    
    /**
     * Holds robot dimensions.
     */
    public static final class RobotDimensions {
        public static final double BASEPLATE_WIDTH = 25.716;
        public static final double LENGTH = 40;
        public static final double WIDTH = 36;
    }

    /**
     * Holds field dimensions (duh).
     * All units are in inches.
     */
    public static final class FieldDimensions {
        public static final double HAB_LVL1_TO_CARGO_SHIP = 172.5;
        public static final double HAB_LVL1_TO_CARGO_SHIP_SIDE = 213.2200787;
        public static final double HAB_LVL1_EDGE_TO_CARGO_SHIP_SIDE = 42.96;
        public static final double HAB_LVL2_LENGTH = 48;
        public static final double CARGOSHIP_FRONT_OFFSET = 10.875;
        public static final double CARGOSHIP_FRONT_OFFSET_SIDE = 12.875;
    }
    public static final RobotSpecs specsLow = new RobotSpecs(42.5, 53, RobotDimensions.BASEPLATE_WIDTH);
    public static final RobotSpecs specsHigh = new RobotSpecs(150, 50, RobotDimensions.BASEPLATE_WIDTH);

    public static final int SHIFT_LOW_TO_HIGH_MAX = Integer.MAX_VALUE;
    public static final int SHIFT_HIGH_TO_LOW_MAX = 48;
  
    public static void init() {
        // Set the motors to follow
        lTalon1.follow(lVictor);
        lTalon2.follow(lVictor);
        rTalon1.follow(rVictor);
        rTalon2.follow(rVictor);

        essieMotorLowUnprotected.setInverted(true);
        essieMotorHighUnprotected.setInverted(false);
		
		leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }
}
