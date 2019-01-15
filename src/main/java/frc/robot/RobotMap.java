/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

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

    // Drive motors
    public static final VictorSPX lVictor = new VictorSPX(0);
    public static final TalonSRX lTalon1 = new TalonSRX(1);
    public static final TalonSRX lTalon2 = new TalonSRX(2);
    public static final VictorSPX rVictor = new VictorSPX(3);
    public static final TalonSRX rTalon1 = new TalonSRX(4);
    public static final TalonSRX rTalon2 = new TalonSRX(5);
  
    public static void init() {
        // Invert victors due to gearbox config
        lVictor.setInverted(true);
        rVictor.setInverted(true);
        // Set the motors to follow
        lTalon1.follow(lVictor);
        lTalon2.follow(lVictor);
        rTalon1.follow(rVictor);
        rTalon2.follow(rVictor);
        // Set all motors into coast mode
        lVictor.setNeutralMode(NeutralMode.Coast);
        rVictor.setNeutralMode(NeutralMode.Coast);
        lTalon1.setNeutralMode(NeutralMode.Coast);
        lTalon2.setNeutralMode(NeutralMode.Coast);
        rTalon1.setNeutralMode(NeutralMode.Coast);
        rTalon2.setNeutralMode(NeutralMode.Coast); 
    }
}
