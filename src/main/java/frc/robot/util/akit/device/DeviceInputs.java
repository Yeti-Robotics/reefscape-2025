package frc.robot.util.akit.device;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// easiest way to avoid typecasting/problems with generics
public interface DeviceInputs extends LoggableInputs {
    @Override
    default void toLog(LogTable table) {}

    @Override
    default void fromLog(LogTable table) {}
}
