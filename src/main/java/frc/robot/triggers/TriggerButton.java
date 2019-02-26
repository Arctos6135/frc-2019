package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

public class TriggerButton extends Button {

    int axis;
    double threshold;
    GenericHID joystick;

    public TriggerButton(GenericHID joystick, int trigger, double threshold) {
        axis = trigger;
        this.joystick = joystick;
        this.threshold = threshold;
    }

    int counter = 0;
    static final int COUNT_REQUIRED = 5;

    @Override
    public boolean get() {
        if(joystick.getRawAxis(axis) >= threshold) {
            if(counter >= COUNT_REQUIRED) {
                return true;
            }
            else {
                counter ++;
                return false;
            }
        }
        else {
            counter = 0;
            return false;
        }
    }

}
