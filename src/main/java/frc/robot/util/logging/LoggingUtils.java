package frc.robot.util.logging;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggingUtils {
    public static <T> void logInputs(String key, InputLoggingIO<T> loggingIO, T loggingData) {
        loggingIO.updateInputs(loggingData);
        // TODO: remove cast
        Logger.processInputs(key, (LoggableInputs) loggingData);
    }
}
