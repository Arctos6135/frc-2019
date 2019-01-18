/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Vision subsystem. Connects directly to the Jetson using NetworkTables.
 */
public class Vision extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    NetworkTableInstance instance;
    NetworkTable table;
    
    NetworkTableEntry visionOnline, visionEnabled, visionEnableSuccess, visionResult;

    /**
     * Creates a new vision subsystem.
     */
    public Vision() {
        super();

        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("roborio-jetson");
        // Get the entries
        visionOnline = table.getEntry("vision-online");
        visionEnabled = table.getEntry("vision-enable");
        visionEnableSuccess = table.getEntry("enable-success");
        visionResult = table.getEntry("horizontal-angle");
    }
    /**
     * Creates a new vision subsystem.
     * 
     * @param name The name of this subsystem
     */
    public Vision(String name) {
        super(name);

        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("roborio-jetson");
        // Get the entries
        visionOnline = table.getEntry("vision-online");
        visionEnabled = table.getEntry("vision-enable");
        visionEnableSuccess = table.getEntry("enable-success");
        visionResult = table.getEntry("horizontal-angle");
    }

    /**
     * Gets whether the Jetson's vision processing is online.
     * @return Whether the Jetson is ready
     */
    public boolean ready() {
        return visionOnline.getBoolean(false);
    }

    /**
     * Calls {@link java.lang.Object#notifyAll()} when the Jetson's vision processing is ready.
     * For example:
     * <p>
     * {@code Vision vision = new Vision();}<br/>
     * {@code vision.notifyWhenReady(this);}<br/>
     * {@code wait();}<br/>
     * </p>
     * @param objectToNotify The object to call {@link java.lang.Object#notifyAll()} on.
     */
    public void notifyWhenReady(Object objectToNotify) {
        visionOnline.addListener(notification -> {
            if(notification.getEntry().getBoolean(false)) {
                objectToNotify.notifyAll();
            }
        }, TableEntryListener.kNew | TableEntryListener.kUpdate);
    }
}
