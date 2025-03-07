package frc.robot.util.akit;

import frc.robot.util.akit.io.InputLoggingIO;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggingUtils {
    public static <T> void logInputs(String key, InputLoggingIO<T> loggingIO, T loggingData) {
        loggingIO.updateInputs(loggingData);
        Logger.processInputs(key, (LoggableInputs) loggingData);
    }
}
