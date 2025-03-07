package frc.robot.util.akit.logging.device.builder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.akit.logging.device.DeviceLogger;
import frc.robot.util.akit.logging.device.TalonFXLogger;
import java.util.*;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;

public class TalonFXLoggingBuilder implements DeviceLoggingBuilder<TalonFX> {
    private final Set<StatusSignal<?>> statusSignals = new HashSet<>();
    private final List<Consumer<LogTable>> loggers = new ArrayList<>();
    private final TalonFX talon;

    private TalonFXLoggingBuilder(TalonFX talon) {
        this.talon = talon;
    }

    private void addStatusSignal(StatusSignal<?> statusSignal) {}

    @Override
    public DeviceLoggingBuilder<TalonFX> withPositionRotations() {
        addStatusSignal(talon.getPosition());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withVelocityRotationsPerSec() {
        addStatusSignal(talon.getVelocity());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withAccelerationRotationsPerSecSq() {
        addStatusSignal(talon.getAcceleration());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withMotorVoltage() {
        addStatusSignal(talon.getMotorVoltage());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withMotorAmps() {
        addStatusSignal(talon.getTorqueCurrent());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withPGain() {
        addStatusSignal(talon.getClosedLoopProportionalOutput());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withDGain() {
        addStatusSignal(talon.getClosedLoopDerivativeOutput());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withError() {
        addStatusSignal(talon.getClosedLoopError());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withPIDOutput() {
        addStatusSignal(talon.getClosedLoopOutput());
        return this;
    }

    @Override
    public DeviceLoggingBuilder<TalonFX> withMotorTemperature() {
        addStatusSignal(talon.getDeviceTemp());
        return this;
    }

    public static TalonFXLoggingBuilder from(TalonFX device) {
        return new TalonFXLoggingBuilder(device);
    }

    @Override
    public DeviceLogger build() {
        return new TalonFXLogger(statusSignals);
    }
}
