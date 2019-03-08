/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * This command literally does nothing.
 */
public class Delay extends TimedCommand {
    /**
     * Literally do nothing for a specified period of time.
     * 
     * @param timeout How long to be useless for
     */
    public Delay(double timeout) {
        super(timeout);
    }
}
