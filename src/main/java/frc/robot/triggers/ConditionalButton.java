package frc.robot.triggers;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.buttons.Button;

/**
 * The {@code ConditionalButton} class is a wrapper around a normal
 * {@code Button}. It is a button that can only be activated when its
 * "condition" evaluates to {@code true}. The "condition" is passed in the form
 * of an {@code AtomicBoolean}, so that it can be modified outside of this
 * class. <br>
 * <br>
 * Example Usage:
 * 
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
    protected boolean invert = false;

    /**
     * Create a new {@code ConditionalButton}.
     * 
     * This button can only be activated when {@code condition.get()} evaluates to
     * {@code true}.
     * 
     * @param button    The button to wrap around
     * @param condition The condition for this button
     */
    public ConditionalButton(Button button, AtomicBoolean condition) {
        this.button = button;
        this.condition = condition;
    }

    /**
     * Create a new {@code ConditionalButton}.
     * 
     * If {@code invert} is {@code false}, then the button can only be activated
     * when {@code condition.get()} is {@code true}. If {@code invert} is
     * {@code true}, then the button can only be activated when
     * {@code condition.get()} is {@code false}.
     * 
     * @param button
     * @param condition
     * @param invert
     */
    public ConditionalButton(Button button, AtomicBoolean condition, boolean invert) {
        this(button, condition);
        this.invert = invert;
    }

    @Override
    public boolean get() {
        // XOR the condition with invert
        // If invert is false, this will not have any effect
        // If invert is true, this will basically invert the condition
        return button.get() && (condition.get() ^ invert);
    }
}
