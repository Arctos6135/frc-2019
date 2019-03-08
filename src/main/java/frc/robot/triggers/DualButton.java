package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A "button" activated when two buttons are pressed at the same time.
 */
public class DualButton extends Button {

    Button a, b;

    /**
     * Constructor.
     * 
     * @param a The first button
     * @param b The second button
     */
    public DualButton(Button a, Button b) {
        this.a = a;
        this.b = b;
    }

    public boolean get() {
        return a.get() && b.get();
    }
}
