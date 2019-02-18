package frc.robot.misc.powermanagement;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.buttons.Trigger;

public class VoltageMonitoringTrigger extends Trigger {

    protected final double threshold;
    protected final double buffer;

    protected boolean activated = false;
    
    public VoltageMonitoringTrigger(double threshold) {
        this(threshold, 0.15);
    }
    public VoltageMonitoringTrigger(double threshold, double buffer) {
        this.threshold = threshold;
        this.buffer = buffer;
    }

    @Override
    public boolean get() {
        // A buffer to prevent the trigger from getting rapidly activated and un-activated due to noise
        double voltage = RobotController.getBatteryVoltage();
        // If not yet activated, return whether the voltage is lower than the threshold
        if(!activated) {
            if(voltage < threshold) {
                activated = true;
                return true;
            }
            else {
                return false;
            }
        }
        // If activated, return true even for voltages higher than the threshold, as long as it is within the buffer
        else {
            if(voltage >= threshold + buffer) {
                activated = false;
                return false;
            }
            else {
                return true;
            }
        }
    }
}
