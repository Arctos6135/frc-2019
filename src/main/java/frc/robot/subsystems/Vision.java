/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryNotification;
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
    @SuppressWarnings("serial")
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

    // This class was created so that notifyWhenReady can remove its callback after it's done.
    static class NotifyWhenReadyCallback implements Consumer<EntryNotification> {
        Object objectToNotify;
        NetworkTableEntry entry;
        int handle = 0;

        public NotifyWhenReadyCallback(Object objectToNotify, NetworkTableEntry entry) {
            this.objectToNotify = objectToNotify;
            this.entry = entry;
        }
        
        public void setHandle(int handle) {
            this.handle = handle;
        }

        public void accept(EntryNotification notif) {
            objectToNotify.notifyAll();
            if(handle != 0) {
                entry.removeListener(handle);
            }
        }
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
        NotifyWhenReadyCallback callback = new NotifyWhenReadyCallback(objectToNotify, visionOnline);
        // Get the handle
        int handle = visionOnline.addListener(callback, TableEntryListener.kNew | TableEntryListener.kUpdate);
        // Give it back to the callback to remove later
        callback.setHandle(handle);
    }

    /**
     * Sets whether vision processing is enabled.
     * 
     * @param enabled Whether vision is enabled
     * @throws VisionException If vision is not ready
     */
    public void setVisionEnabled(boolean enabled) throws VisionException {
        setVisionEnabled(enabled, false, 0);
    }
    /**
     * Sets whether vision processing is enabled.
     * 
     * @param enabled Whether vision is enabled
     * @param check Whether or not to check if the attempt was successful
     * @throws VisionException If vision is not ready, or enable not successful, or timeout
     */
    public void setVisionEnabled(boolean enabled, boolean check, int timeout) throws VisionException {
        if(!ready()) {
            // Oh no! It's busted
            throw new VisionException("Vision is offline!");
        }
        
        // Keep a handle to the listener to remove it later
        int handle = 0;
        if(enabled && check) {
            // Set up the listener only if enable
            handle = visionEnableSuccess.addListener(notification -> {
                this.notifyAll();
            }, TableEntryListener.kNew | TableEntryListener.kUpdate);
        }

        visionEnabled.setBoolean(enabled);

        // Wait for value update
        // Only do this if enable and check
        if(enabled && check) {
            try {
                wait(timeout);
            }
            catch(InterruptedException e) {
                e.printStackTrace();
            }
            
            visionEnableSuccess.removeListener(handle);
            if(visionEnableSuccess.getBoolean(false)) {
                throw new VisionException("Vision enable failed!");
            }
        }
    }

    public boolean getVisionEnabled() {
        return visionEnabled.getBoolean(false);
    }
}
