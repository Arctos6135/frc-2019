package frc.robot.triggers;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.buttons.Button;

/**
 * The {@code ConditionalButton} class is a wrapper around a normal {@code Button}.
 * It is a button that can only be activated when its "condition" evaluates to {@code true}.
 * The "condition" is passed in the form of an {@code AtomicBoolean}, so that it can be
 * modified outside of this class.
 * <br><br>
 * Example Usage:
 * <pre>
 * AtomicBoolean allowInput = new AtomicBoolean(true);
 * Button b = new ConditionalButton(new JoystickButton(joystick, buttonNumber), allowInput);
 * b.whenPressed(new SomeCommand()); // After this point, SomeCommand will run when the button is pressed
 * // ...
 * allowInput.set(false); // After this point, SomeCommand will not run, even if the button is pressed
 * </pre>
 */
public class ConditionalButton extends Button {

    protected Button button;
    protected AtomicBoolean condition;

    public ConditionalButton(Button button, AtomicBoolean condition) {
        this.button = button;
        this.condition = condition;
    }

    @Override
    public boolean get() {
        return button.get() && condition.get();
    }
}
