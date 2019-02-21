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
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RobotLogger {
    static Handler fileHandler;
    static Formatter formatter;

    static Logger logger;

    static boolean isInitialized = false;

    public static void init() throws IOException {
        // Get logger for robot class
        logger = Logger.getLogger(Robot.class.getName());
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
        formatter = new SimpleFormatter();
        fileHandler.setFormatter(formatter);
        logger.addHandler(fileHandler);

        isInitialized = true;
    }

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
}
