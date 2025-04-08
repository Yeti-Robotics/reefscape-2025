package frc.robot.util.akit.device;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class DeviceLoggingRegistry {
    private static final DeviceLoggingRegistry INSTANCE = new DeviceLoggingRegistry();
    private final ArrayList<LoggingEntry<?>> loggers = new ArrayList<>();
    private boolean disable = false;

    private record LoggingEntry<T extends DeviceInputs>(
            String key, DeviceLogger<T> logger, T inputs) {
        public T getUpdatedInputs() {
            logger.updateInputs(inputs);
            return inputs;
        }
    }

    protected <T extends DeviceInputs> void addLoggerWithInputs(
            String key, DeviceLogger<T> logger, T inputs) {
        if (!disable) {
            loggers.add(new LoggingEntry<>(key, logger, inputs));
        }
    }

    /**
     * @apiNote call this method as frequently as you would like to log device data
     */
    public void updateDeviceLogging() {
        if (disable) return;

        for (LoggingEntry<?> entry : loggers) {
            Logger.processInputs(entry.key, entry.getUpdatedInputs());
        }
    }

    /**
     * @apiNote disables logging for ALL devices, use only if you don't want to log anything
     */
    public void disableDeviceLogging() {
        disable = true;
        loggers.clear();
    }

    public static DeviceLoggingRegistry get() {
        return INSTANCE;
    }
}
