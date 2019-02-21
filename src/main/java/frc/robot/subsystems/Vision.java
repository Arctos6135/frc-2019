/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Vision subsystem. Connects directly to the Jetson using NetworkTables.
 */
public class Vision extends Subsystem {

    static final double CAMERA_Y_OFFSET = 15;

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
    public NetworkTable table;
    
    public NetworkTableEntry visionOnline, visionEnabled, visionEnableSuccess, visionResult;
    public NetworkTableEntry angleOffset, xOffset, yOffset;

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

        angleOffset = table.getEntry("angle-offset");
        xOffset = table.getEntry("x-offset");
        yOffset = table.getEntry("y-offset");
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

        angleOffset = table.getEntry("angle-offset");
        xOffset = table.getEntry("x-offset");
        yOffset = table.getEntry("y-offset");
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
        int handle = visionOnline.addListener(callback, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
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
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }

        visionEnabled.setBoolean(enabled);

        // Wait for value update
        // Only do this if enable and check
        if(enabled && check) {
            try {
                synchronized(this) {
                    wait(timeout);
                }
            }
            catch(InterruptedException e) {
                e.printStackTrace();
            }
            
            visionEnableSuccess.removeListener(handle);
            if(!visionEnableSuccess.getBoolean(false)) {
                throw new VisionException("Vision enable failed!");
            }
        }
        // Set the result entry to NaN just to be sure
        visionResult.setNumber(Double.NaN);
    }

    /**
     * Returns whether or not vision is enabled.
     * @return Whether vision is enabled.
     */
    public boolean getVisionEnabled() {
        return visionEnabled.getBoolean(false);
    }

    /**
     * Returns the result of the vision tracking.
     * A negative angle indicates that the target is on the left and vice versa.
     * 
     * @return The horizontal angle, in degrees, of the tracked object. A value of NaN indcates that the object is not found.
     * @throws VisionException If vision is not ready
     */
    public double getVisionResult() throws VisionException {
        if(!ready()) {
            throw new VisionException("Vision is offline!");
        }

        return visionResult.getDouble(Double.NaN);
    }
    /**
     * Returns the orientation of the target, with respect to the robot.
     * @return The angle offset of the target
     * @throws VisionException If vision is not ready
     */
    public double getTargetAngleOffset() throws VisionException {
        if(!ready()) {
            throw new VisionException("Vision is offline!");
        }

        return angleOffset.getDouble(Double.NaN);
    }
    /**
     * Returns the X coordinate of the centre of the target, with respect to the robot.
     * @return The X coordinate offset of the target
     * @throws VisionException If vision is not ready
     */
    public double getTargetXOffset() throws VisionException {
        if(!ready()) {
            throw new VisionException("Vision is offline!");
        }

        return xOffset.getDouble(Double.NaN);
    }
    /**
     * Returns the Y coordinate of the centre of the target, with respect to the robot.
     * @return The Y coordinate offset of the target
     * @throws VisionException If vision is not ready
     */
    public double getTargetYOffset() throws VisionException {
        if(!ready()) {
            throw new VisionException("Vision is offline!");
        }

        return yOffset.getDouble(Double.NaN) - CAMERA_Y_OFFSET;
    }
 
    /**
     * Adds a callback function to be called whenever there's a new vision tracking result.
     * @param callback The callback
     * @return The handle to the callback, used for removal
     */
    public int addResultCallback(Consumer<Double> callback) {
        return visionResult.addListener(notification -> {
            // Add the listener and return the handle
            callback.accept(notification.getEntry().getDouble(Double.NaN));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    /**
     * Removes a callback function for the vision tracking result.
     * @param handle The handle to the callback
     */
    public void removeResultCallback(int handle) {
        visionResult.removeListener(handle);
    }

    /**
     * Sends a signal to the Jetson for it to gracefully shutdown.
     * <b>Use with extreme care. Once the Jetson is shut down, there is no way to turn
     * it back on using software.</b>
     */
    public void shutdownJetson() {
        NetworkTableEntry shutdown = table.getEntry("shutdown");
        shutdown.setBoolean(true);
        visionOnline.setBoolean(false);
    }

    public void restartServer() {
        NetworkTableEntry restart = table.getEntry("restart-server");
        restart.setBoolean(true);
    }
}
