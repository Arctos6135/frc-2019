package frc.robot.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class VisiSight extends DigitalInput {

	public VisiSight(int channel) {
		super(channel);
	}

    public boolean isBlocked() {
        return super.get();
    }
}