/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;

public class Drivetrain extends Subsystem {
    
    public static class Position {
        public double x;
        public double y;
        public double heading;

        public Position(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // For position integration
    private Position position = new Position(0, 0, 0);
    private double lastLeft, lastRight, initHeading;

    // For calculating acceleration
	private double leftLastRate = 0, rightLastRate = 0;
    private double lastTime;

    public Drivetrain() {
        // Reset variables for position integration
        lastLeft = getLeftDistance();
        lastRight = getRightDistance();
        initHeading = getHeading();

        setMotors(0, 0);
        setGear(Gear.LOW);
        resetHeading();
        setNeutralMode(NeutralMode.Coast);
    }
    public Drivetrain(String name) {
        super(name);

        lastLeft = getLeftDistance();
        lastRight = getRightDistance();
        initHeading = getHeading();

        setMotors(0, 0);
        setGear(Gear.LOW);
        resetHeading();
        setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new TeleopDrive());
    }

    @Override
    public void periodic() {
        double left = getLeftDistance();
        double right = getRightDistance();
        // Change in distance for left and right wheels
        double dl = left - lastLeft;
        double dr = right - lastRight;
        // Change in euclidean distance
        double ds = (dl + dr) / 2;
        // Heading is absolute as obtained with the gyro
        double heading = getHeading() - initHeading;

        double dx = ds * Math.cos(Math.toRadians(heading));
        double dy = ds * Math.sin(Math.toRadians(heading));

        lastLeft = left;
        lastRight = right;

        position.x += dx;
        position.y += dy;
        position.heading = heading;
    }

    /**
     * Converts an angle in degrees to be in the range (-180, 180].
     * 
     * @param angle The angle to constrain
     * @return The constrained angle
     */
    public static double constrainAngle(double angle) {
        if(angle <= 180.0 && angle > -180.0)
            return angle;
        while(angle > 180.0) {
            angle -= 360.0;
        }
        while(angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    // Having a speed multiplier allows for easy adjustment of top speeds
    private double speedMultiplier = 1.0;

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
    
    // Stores the previous left and right output values
    // Used by TeleopDrive for ramping
    double prevLeft = 0;
    double prevRight = 0;

    public double getPrevLeft() {
        return prevLeft;
    }
    public double getPrevRight() {
        return prevRight;
    }

    /**
     * Sets the left and right side motors of the drivetrain. 
     * The input values are first multiplied by the speed multiplier (see {@link #setSpeedMultiplier(double)}), 
     * and then constrained to [-1, 1].
     * @param left The left side motors percent output
     * @param right The right side motors percent output
     */
    public void setMotors(double left, double right) {
        prevLeft = left;
        prevRight = right;
        RobotMap.lVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, -left * speedMultiplier)));
        // Invert right side
        RobotMap.rVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, right * speedMultiplier)));
    }
    
    public void setLeftMotor(double output) {
        prevLeft = output;
        RobotMap.lVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, -output * speedMultiplier)));
    }
    public void setRightMotor(double output) {
        prevRight = output;
        RobotMap.rVictor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, output * speedMultiplier)));
    }
	
	// Encoders
	/** 
	 * Reset the left and right encoders
	*/
	public void resetEncoders() {
		RobotMap.leftEncoder.reset();
		RobotMap.rightEncoder.reset();
	}

	/**
	 * Gets the distance travelled by the left encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of the {@code Encoder} class
	 * @return The distance travelled by the left encoder
	 */
	public double getLeftDistance() {
		return RobotMap.leftEncoder.getDistance();
	}

	/**
	 * Gets the distance travelled by the right encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of the {@code Encoder} class
	 * @return The distance travelled by the right encoder
	 */
	public double getRightDistance() {
		return RobotMap.rightEncoder.getDistance();
	}

	/**
	 * Gets the speed reading of the left encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the {@code Encoder} class
	 * @return The speed of the left encoder
	 */
	public double getLeftSpeed() {
		return RobotMap.leftEncoder.getRate();
	}

	/**
	 * Gets the speed reading of the right encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the {@code Encoder} class
	 * @return The speed of the right encoder
	 */
	public double getRightSpeed() {
		return RobotMap.rightEncoder.getRate();
	}

	/**
     * Calculates the acceleration of both sides.<br>
     * <br>
     * Instead of being read directly from the encoder, the acceleration is calculated by deriving the result
     * of {@link #getLeftSpeed()} and {@link #getRightSpeed()}. The derivation is done only when this method
     * is called, therefore the result will be more accurate if this method was called a short time ago.
     * However, please note that two subsequent calls right after each other may yield a result of 0, as the
     * last known encoder rate from {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} may not have time
     * to update.
     * @return An array containing the acceleration of both sides, with element 0 being the left and element 1 
     * being the right
     */
	public double[] getAccelerations() {
    	double dt = Timer.getFPGATimestamp() - lastTime;
    	double leftRate = RobotMap.leftEncoder.getRate();
    	double rightRate = RobotMap.rightEncoder.getRate();
    	double leftAccel = (leftRate - leftLastRate) / dt;
    	double rightAccel = (rightRate - rightLastRate) / dt;
    	leftLastRate = leftRate;
    	rightLastRate = rightRate;
    	lastTime = Timer.getFPGATimestamp();
    	return new double[] { leftAccel, rightAccel };
	}
	
	/**
	 * Gets the left {@code Encoder} object
	 * @return The left encoder
	 */
	public Encoder getLeftEncoder() {
		return RobotMap.leftEncoder;
	}

	/**
	 * Gets the right {@code Encoder} object
	 * @return The right encoder
	 */
	public Encoder getRightEncoder() {
		return RobotMap.rightEncoder;
	}

    public enum Gear {
        LOW, HIGH;
    }

    private Gear gear;
    /**
     * Sets the drivetrain gearboxes to be in either low or high gear.
     * 
     * Note that the low and high gear values may be switched depending on how the pneumatics are wired!
     * @param gear The new gear, {@link Gear#LOW} or {@link Gear#HIGH}.
     */
    public void setGear(Gear gear) {
        if(gear == Gear.HIGH) {
            RobotMap.gearShifter.set(DoubleSolenoid.Value.kForward);
            Robot.drivetrainGearEntry.setString("HIGH");
        }
        else {
            RobotMap.gearShifter.set(DoubleSolenoid.Value.kReverse);
            Robot.drivetrainGearEntry.setString("LOW");
        }
        this.gear = gear;
    }
    /**
     * Gets the gear the drivetrain gearboxes are in.
     * 
     * Note that the low and high gear values may be switched depending on how the pneumatics are wired!
     * @return The gear, {@link Gear#LOW} or {@link Gear#HIGH}.
     */
    public Gear getGear() {
        return gear;
    }

    /**
     * Returns the heading (yaw) of the robot.
     * The yaw is first passed into {@link #constrainAngle(double)}, so it is in the range (-180, 180].
     * @return The yaw of the robot
     */
    public double getHeading() {
        return constrainAngle(RobotMap.ahrs.getYaw());
    }
    /**
     * Resets the heading (yaw) of the robot.
     */
    public void resetHeading() {
        RobotMap.ahrs.reset();
    }

    NeutralMode neutralMode;
    /**
     * Sets the neutral mode (brake or coast) of all the drivetrain motors.
     */
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
        
        RobotMap.lVictor.setNeutralMode(mode);
        RobotMap.lTalon1.setNeutralMode(mode);
        RobotMap.lTalon2.setNeutralMode(mode);

        RobotMap.rVictor.setNeutralMode(mode);
        RobotMap.rTalon1.setNeutralMode(mode);
        RobotMap.rTalon2.setNeutralMode(mode);
    }
    /**
     * Gets the neutral mode of all the drivetrain motors.
     */
    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    /**
     * Returns the estimated position of the robot.
     * 
     * <p>
     * {@link #startIntegration(int)} must be called before this.
     * </p>
     * 
     * @return The estimated position.
     */
    public Position getPosition() {
        return new Position(position.x, position.y, position.heading);
    }

    /**
     * Resets the estimated position of the robot.
     */
    public void resetPosition() {
        position.x = 0;
        position.y = 0;
        position.heading = 0;
    }

    public class Gyro extends GyroBase {

        @Override
        public void calibrate() {
            // Not implemented
        }

        @Override
        public void reset() {
            resetHeading();
        }

        @Override
        public double getAngle() {
            return getHeading();
        }

        @Override
        public double getRate() {
            return RobotMap.ahrs.getRate();
        }

    }
}
