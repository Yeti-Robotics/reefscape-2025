package frc.robot.util.akit.logging.device;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.List;

public class TalonFXLoggingBuilder implements DeviceLoggingBuilder<TalonFX> {
    private final List<StatusSignal<?>> statusSignals = new ArrayList<>();
}
