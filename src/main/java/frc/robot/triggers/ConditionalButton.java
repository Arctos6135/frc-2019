package frc.robot.triggers;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.buttons.Button;

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
