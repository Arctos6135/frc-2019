package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A "button" activated when all of multiple buttons are pressed at the same time.
 */
public class AllOfButton extends Button {

    Button[] buttons;

    /**
     * Constructor.
     * 
     * @param buttons The buttons that need to be pressed
     */
    public AllOfButton(Button... buttons) {
        this.buttons = buttons;
    }
    
    @Override
    public boolean get() {
        for(Button b : buttons) {
            if(!b.get()) {
                return false;
            }
        }
        return true;
    }
}
