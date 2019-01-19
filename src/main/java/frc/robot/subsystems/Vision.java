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

    /**
     * Indicates that something has gone wrong with vision, typically the communications to the Jetson.
     */
    public class VisionException extends Exception {
        public VisionException() {
            super();
        }
        public VisionException(String msg) {
            super(msg);
        }
    }

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

    /**
     * Sets whether vision processing is enabled.
     * 
     * @param enabled Whether vision is enabled
     * @throws VisionException If vision is not ready
     */
    public void setVisionEnabled(boolean enabled) throws VisionException {
        setVisionEnabled(enabled, false);
    }
    /**
     * Sets whether vision processing is enabled.
     * 
     * @param enabled Whether vision is enabled
     * @param check 
     * @throws VisionException
     */
    public void setVisionEnabled(boolean enabled, boolean check) throws VisionException {
        if(!ready()) {
            // Oh no! It's busted
            throw new VisionException("Vision is offline!");
        }
        
        if(enabled) {
            // Set up the listener only if enable
            visionEnableSuccess.addListener(notification -> {
                this.notifyAll();
            }, TableEntryListener.kNew | TableEntryListener.kUpdate);
        }

        visionEnabled.setBoolean(enabled);

        // Wait for value update
        // Only do this if enable and check
        if(enabled && check) {
            try {
                wait();
            }
            catch(InterruptedException e) {
                e.printStackTrace();
            }

            if(visionEnableSuccess.getBoolean(false)) {
                throw new VisionException("Vision enable failed!");
            }
        }
    }

    public boolean getVisionEnabled() {
        return visionEnabled.getBoolean(false);
    }
}
