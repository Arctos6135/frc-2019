/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class OperateHank extends InstantCommand {

    public static final int PUSH_OUT = 1;
    public static final int RETRACT = 2;

    private int direction;

    /**
     * Add your docs here.
     */
    public OperateHank(int direction) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.hank);
        this.direction = direction;
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        if(direction == PUSH_OUT) {
            Robot.hank.pushOut();
        }
        else {
            Robot.hank.retract();
        }
    }

}
