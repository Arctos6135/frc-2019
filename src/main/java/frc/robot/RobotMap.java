/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.misc.BeautifulRobotDriver;
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

    // Encoder constants
	public static final int WHEEL_DIAMETER = 6; //INCHES
	public static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER*Math.PI;
	public static final double DRIVE_ENCODER_PPR = 256;
    public static final double DISTANCE_PER_PULSE = WHEEL_CIRCUMFRENCE/DRIVE_ENCODER_PPR*5/48;
    
    // Drivetrain parameters
    public static final double BASEPLATE_WIDTH = 25.716;

    public static final DoubleSolenoid hankSolenoid = new DoubleSolenoid(2, 3);
    public static final DoubleSolenoid gearShifter = new DoubleSolenoid(0, 1);

    // Drive motors
    public static final VictorSPX lVictor = new VictorSPX(0);
    public static final TalonSRX lTalon1 = new TalonSRX(1);
    public static final TalonSRX lTalon2 = new TalonSRX(2);
    public static final VictorSPX rVictor = new VictorSPX(3);
    public static final TalonSRX rTalon1 = new TalonSRX(4);
	public static final TalonSRX rTalon2 = new TalonSRX(5);

    // Essie motors
    public static final VictorSP essieMotorLowUnprotected = new VictorSP(0);
    public static final VictorSP essieMotorHighUnprotected = new VictorSP(1);
    public static final ProtectedMotor essieMotorLow = new ProtectedMotor((speed) -> {
        essieMotorLowUnprotected.set(speed);
    }, 4, 25, 1, () -> {
        OI.errorRumbleOperatorMajor.execute();
    });
    public static final ProtectedMotor essieMotorHigh = new ProtectedMotor((speed) -> {
        essieMotorHighUnprotected.set(speed);
    }, 5, 25, 1, () -> {
        OI.errorRumbleOperatorMajor.execute();
    });
    public static final DigitalInput essiePhotoElectric = new DigitalInput(4);

    public static final DigitalInput hankSwitch = new DigitalInput(5);

    // navX
    public static final AHRS ahrs = new AHRS(I2C.Port.kOnboard);
	public static Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	public static Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	
    public static final BeautifulRobotDriver beautifulRobotDriver = new BeautifulRobotDriver(Port.kMXP);
    
    public static final RobotSpecs specs = new RobotSpecs(120, 80, BASEPLATE_WIDTH);
  
    public static void init() {
        // Set the motors to follow
        lTalon1.follow(lVictor);
        lTalon2.follow(lVictor);
        rTalon1.follow(rVictor);
        rTalon2.follow(rVictor);
		
		leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }
}
