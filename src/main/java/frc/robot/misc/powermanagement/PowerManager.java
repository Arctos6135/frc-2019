package frc.robot.misc.powermanagement;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.misc.RobotLogger;

@SuppressWarnings("resource")
public final class PowerManager {
    private PowerManager() {}

    public static void startCompressorPowerManagement(double lowestVoltage) {
        VoltageMonitoringTrigger trig = new VoltageMonitoringTrigger(lowestVoltage);
        trig.whenActive(new InstantCommand(() -> {
            RobotMap.compressor.stop();
            RobotLogger.logWarning("Compressor stopped due to voltage too low");
        }));
        trig.whenInactive(new InstantCommand(() -> {
            RobotMap.compressor.start();
            RobotLogger.logInfo("Compressor restarted after voltage has recovered");
        }));
    }
    
    private static double regularMultiplier = Robot.drivetrain.getSpeedMultiplier();
    public static void startDrivetrainPowerManagement(double lowestVoltage, double multiplierWhenActivated) {
        VoltageMonitoringTrigger trig = new VoltageMonitoringTrigger(lowestVoltage);
        trig.whenActive(new InstantCommand(() -> {
            regularMultiplier = Robot.drivetrain.getSpeedMultiplier();
            Robot.drivetrain.setSpeedMultiplier(multiplierWhenActivated);
            RobotLogger.logWarning("Drivetrain max power output reduced to prevent brownout");
        }));
        trig.whenInactive(new InstantCommand(() -> {
            Robot.drivetrain.setSpeedMultiplier(regularMultiplier);
            RobotLogger.logInfo("Drivetrian power back to normal after voltage has recovered");
        }));
    }

    public static void startEssiePowerManagement() {
        Robot.essie.setDisableCompressorWhenActive(true);
    }
}
