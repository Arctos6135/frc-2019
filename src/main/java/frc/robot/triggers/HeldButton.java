package frc.robot.triggers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * A "button" that activates after being held down for a certain amount of time.
 */
public class HeldButton extends Button {

    Button button;
    double duration;

    /**
     * Constructor.
     * 
     * @param button The button that needs to be held down
     * @param duration The duration it needs to be held down for (in seconds)
     */
    public HeldButton(Button button, double duration) {
        this.button = button;
        this.duration = duration;
    }

    double pressedAt = Double.NaN;

    @Override
    public boolean get() {
        // If button is pressed down:
        if(button.get()) {
            // Pressed down since is NaN (button not pressed down before), set the value
            if(Double.isNaN(pressedAt)) {
                pressedAt = Timer.getFPGATimestamp();
            }
            // Return whether the button has been pressed for more than the specified duration
            return Timer.getFPGATimestamp() - pressedAt >= duration;
        }
        else {
            // If the button is not pressed, reset the last pressed down time
            pressedAt = Double.NaN;
            return false;
        }
    }
}
