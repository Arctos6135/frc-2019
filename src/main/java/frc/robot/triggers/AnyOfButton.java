package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A "button" activated when any of multiple buttons are pressed at the same time.
 */
public class AnyOfButton extends Button {

    Button[] buttons;

    /**
     * Constructor.
     * 
     * @param buttons The buttons that could be pressed
     */
    public AnyOfButton(Button... buttons) {
        this.buttons = buttons;
    }
    
    @Override
    public boolean get() {
        for(Button b : buttons) {
            if(b.get()) {
                return true;
            }
        }
        return false;
    }
}
