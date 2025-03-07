package frc.robot.util.akit.logging.device;

import java.util.*;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DeviceLogger implements LoggableInputs {
    protected final Collection<Runnable> loggingCallbacks;

    public DeviceLogger(Collection<Runnable> loggingCallbacks) {
        this.loggingCallbacks = loggingCallbacks;
    }

    @Override
    public void toLog(LogTable table) {
        for (Runnable loggingCallback : loggingCallbacks) {
            loggingCallback.run();
        }
    }

    @Override
    public void fromLog(LogTable table) {}
}
