package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A "button" activated when two buttons are pressed at the same time.
 */
public class DualButton extends Button {

    int code1, code2;
    GenericHID controller1, controller2;

    /**
     * Constructor.
     * 
     * @param controller1 The controller of the first button
     * @param button1 The first button's code
     * @param controller2 The controller of the second button
     * @param button2 The second button's code
     */
    public DualButton(GenericHID controller1, int button1, GenericHID controller2, int button2) {
        this.code1 = button1;
        this.code2 = button2;
        this.controller1 = controller1;
        this.controller2 = controller2;
    }

    public boolean get() {
        return controller1.getRawButton(code1) && controller2.getRawButton(code2);
    }
}
