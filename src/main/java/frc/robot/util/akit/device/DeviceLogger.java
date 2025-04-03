package frc.robot.util.akit.device;

import frc.robot.util.akit.io.InputLoggingIO;

public interface DeviceLogger<T extends DeviceInputs> extends InputLoggingIO<T> {
    double CONNECTED_DEBOUNCE_TIME = 0.5;
}
