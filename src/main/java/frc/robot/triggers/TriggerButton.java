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

    @Override
    public boolean get() {
        return joystick.getRawAxis(axis) >= threshold;
    }

}
