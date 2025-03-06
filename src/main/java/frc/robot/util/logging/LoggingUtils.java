package frc.robot.util.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.util.logging.io.InputLoggingIO;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggingUtils {
    public static <T extends LoggableInputs> void logInputs(String key, InputLoggingIO<T> loggingIO, T loggingData) {
        loggingIO.updateInputs(loggingData);
        Logger.processInputs(key, loggingData);
    }
}
