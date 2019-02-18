package frc.robot.misc.powermanagement;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

@SuppressWarnings("resource")
public final class PowerManager {
    private PowerManager() {}

    public static void startCompressorPowerManagement(double lowestVoltage) {
        VoltageMonitoringTrigger trig = new VoltageMonitoringTrigger(lowestVoltage);
        trig.whenActive(new InstantCommand(() -> {
            RobotMap.compressor.stop();
        }));
        trig.whenInactive(new InstantCommand(() -> {
            RobotMap.compressor.start();
        }));
    }
    
    private static double regularMultiplier = Robot.drivetrain.getSpeedMultiplier();
    public static void startDrivetrainPowerManagement(double lowestVoltage, double multiplierWhenActivated) {
        VoltageMonitoringTrigger trig = new VoltageMonitoringTrigger(lowestVoltage);
        trig.whenActive(new InstantCommand(() -> {
            regularMultiplier = Robot.drivetrain.getSpeedMultiplier();
            Robot.drivetrain.setSpeedMultiplier(multiplierWhenActivated);
        }));
        trig.whenInactive(new InstantCommand(() -> {
            Robot.drivetrain.setSpeedMultiplier(regularMultiplier);
        }));
    }

    public static void startEssiePowerManagement() {
        Robot.essie.setDisableCompressorWhenActive(true);
    }
}
