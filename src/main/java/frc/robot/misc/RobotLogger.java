package frc.robot.misc;

import java.io.File;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * This class logs messages to a log file and/or the Driver
 * Station/SmartDashboard.
 */
public class RobotLogger {
    static Handler fileHandler;
    static Formatter formatter;
    static Logger logger;

    static DateFormat dateFormat;

    static boolean isInitialized = false;

    static class RobotLoggerFormatter extends Formatter {

        static DateFormat format = new SimpleDateFormat("(yyyy/MM/dd hh:mm:ss)");

        @Override
        public String format(LogRecord record) {
            StringBuilder builder = new StringBuilder(format.format(new Date()));
            builder.append(" [").append(record.getLevel().toString()).append("]: ").append(record.getMessage()).append("\n");
            return builder.toString();
        }
    }

    /**
     * Initializes the logger. If not initialized, logging will not have any affect.
     */
    public static void init() throws IOException {
        // Get logger for robot class
        logger = Logger.getLogger(Robot.class.getName());
        logger.setUseParentHandlers(false);
        // Logging level fine
        logger.setLevel(Level.FINE);
        // Get a date string (for naming the log file)
        DateFormat format = new SimpleDateFormat("yyyy_MM_dd-hh_mm_ss");
        Date date = new Date();
        
        // Create the log directory if it does not exist
        File logDir = new File("/home/lvuser/frc2019-logs");
        logDir.mkdirs();
        // Create handler and formatter
        fileHandler = new FileHandler("/home/lvuser/frc2019-logs/" + format.format(date) + ".log");
        formatter = new RobotLoggerFormatter();
        fileHandler.setFormatter(formatter);
        logger.addHandler(fileHandler);

        isInitialized = true;
    }

    /**
     * Sets the logging level. Generally, levels SEVERE and WARNING contain error and warning messages;
     * level INFO contains basic info such as enabled/disabled state; level FINE contains even more info
     * such as option changes; level FINER contains all info, including joystick data.
     * 
     * @param level The logging level.
     */
    public static void setLevel(Level level) {
        logger.setLevel(level);
    }

    public static void logError(String error) {
        if(isInitialized) {
            SmartDashboard.putString("Last Error", error);
            DriverStation.reportError(error, false);
            logger.severe(error);
        }
    }
    
    public static void logWarning(String warning) {
        if(isInitialized) {
            SmartDashboard.putString("Last Warning", warning);
            DriverStation.reportWarning(warning, false);
            logger.warning(warning);
        }
    }

    public static void logInfo(String info) {
        if(isInitialized) {
            logger.info(info);
        }
    }

    public static void logInfoFine(String infoFine) {
        if(isInitialized) {
            logger.fine(infoFine);
        }
    }

    public static void logInfoFiner(String infoFiner) {
        if(isInitialized) {
            logger.finer(infoFiner);
        }
    }

    public static void flush() {
        fileHandler.flush();
    }
}