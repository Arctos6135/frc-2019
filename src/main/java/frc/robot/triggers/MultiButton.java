package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A "button" activated when multiple buttons are pressed at the same time.
 */
public class MultiButton extends Button {

    Button[] buttons;

    /**
     * Constructor.
     * 
     * @param buttons
     */
    public MultiButton(Button... buttons) {
        this.buttons = buttons;
    }
    
    @Override
    public boolean get() {
        boolean result = true;
        for(Button b : buttons) {
            result &= b.get();
        }
        return result;
    }
}
