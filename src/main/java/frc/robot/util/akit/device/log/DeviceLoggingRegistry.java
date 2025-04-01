package frc.robot.util.akit.device.log;

import frc.robot.util.akit.device.inputs.DeviceInputs;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class DeviceLoggingRegistry {
    private static final DeviceLoggingRegistry INSTANCE = new DeviceLoggingRegistry();
    private final Map<String, LoggingEntry<?>> loggers = new HashMap<>();

    private record LoggingEntry<T extends DeviceInputs>(DeviceLogger<T> logger, T inputs) {
        public T getUpdatedInputs() {
            logger.updateInputs(inputs);
            return inputs;
        }
    }

    public <T extends DeviceInputs> void addLoggerWithInputs(String key, DeviceLogger<T> logger, T inputs) {
        loggers.put(key, new LoggingEntry<>(logger, inputs));
    }

    public void updateDeviceLogging() {
        for (Map.Entry<String, LoggingEntry<?>> entry : loggers.entrySet()) {
            Logger.processInputs(entry.getKey(), entry.getValue().getUpdatedInputs());
        }
    }

    public static DeviceLoggingRegistry get() {
        return INSTANCE;
    }
}
