package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A button that can be activated when the value of some analog axis exceed a
 * certain amount.
 */
public class AnalogButton extends Button {

    private int axis;
    private double threshold;
    private GenericHID joystick;
    private boolean reverse;

    /**
     * Constructor.
     * 
     * @param joystick  The controller
     * @param trigger   The specific axis to read from
     * @param threshold The threshold that needs to be reached for this button to
     *                  activate
     */
    public AnalogButton(GenericHID joystick, int trigger, double threshold) {
        axis = trigger;
        this.joystick = joystick;
        this.threshold = threshold;
    }

    /**
     * Constructor.
     * 
     * @param joystick  The controller
     * @param trigger   The specific axis to read from
     * @param threshold The threshold that needs to be reached for this button to
     *                  activate
     * @param reverse   If true, the value must be less than the threshold for the
     *                  button to activate
     */
    public AnalogButton(GenericHID joystick, int trigger, double threshold, boolean reverse) {
        axis = trigger;
        this.joystick = joystick;
        this.threshold = threshold;
        this.reverse = reverse;
    }

    int counter = 0;
    static final int COUNT_REQUIRED = 5;

    @Override
    public boolean get() {
        boolean exceeded = reverse ? joystick.getRawAxis(axis) <= threshold : joystick.getRawAxis(axis) >= threshold;
        if (exceeded) {
            if (counter >= COUNT_REQUIRED) {
                return true;
            } else {
                counter++;
                return false;
            }
        } else {
            counter = 0;
            return false;
        }
    }

}
