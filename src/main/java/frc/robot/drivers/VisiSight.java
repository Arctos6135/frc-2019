package frc.robot.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class VisiSight {
    DigitalInput sensor;
    public VisiSight(DigitalInput sensor) {
        this.sensor = sensor;
    }
    public boolean isBlocked() {
        return sensor.get();
    }
}