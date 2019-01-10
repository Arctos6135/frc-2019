package frc.robot.misc;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.lang.Thread;

public class Rumble implements Runnable {
	/**
	 * This class (and it's only function) rumbles the controller anyway you like it.
	 * Use cases include notifying the driver of certain values
	 * It sets the rumble to the intensity you want, and then waits a certain time before turning it off
	 * To run this function in another part of the project, and have it done asynchronously: do it like this
	 * 
		 * ExecutorService executor = Executors.newSingleThreadExecutor(); // only needed once
		 * Rumble myRumble = new Rumble(values); // set preset Rumbles that you will be using beforehand with their values ; only needed once
		 * executor.execute(myRumble); // whenever you need to rumble the controller, run this, passing in the Rumble object you created before
		 * 
	 @param controller The XboxController object that you're rumbling on
	 @param side A string value that details the side to rumble: "left", "right", or "both"
	 @param intensity The intensity to rumble at between 0 and 1
		
	 */

	private static final int sleepTime = 300;
	private XboxController controller;
	private String side;
	private float intensity;

	public void Rumble (XboxController controller, String side, float intensity) {
		this.controller = controller;
		this.side = side;
		this.intensity = intensity;
	}


	@Override
	public void run() {
		try { // run() can't throw errors and Thread.sleep() can throw InterruptedException
			if (side == "right")  {
				controller.setRumble(RumbleType.kRightRumble, intensity);
				Thread.sleep(sleepTime);
				controller.setRumble(RumbleType.kRightRumble, 0.0);
			} else if (side == "left") {
				controller.setRumble(RumbleType.kLeftRumble, intensity);
				Thread.sleep(sleepTime);
				controller.setRumble(RumbleType.kLeftRumble, 0.0);
			} else if (side == "both") {
				controller.setRumble(RumbleType.kRightRumble, intensity);
				controller.setRumble(RumbleType.kLeftRumble, intensity);
				Thread.sleep(sleepTime);
				controller.setRumble(RumbleType.kRightRumble, 0.0);
				controller.setRumble(RumbleType.kLeftRumble, 0.0);
			}
		} catch (InterruptedException e) {
			System.out.println("Rumble's sleep method returned an InterruptedException. Hope that never happens.");
		}
	}
}
