package frc.robot.util.akit.logging.device.builder;

import frc.robot.util.akit.logging.device.DeviceLogger;

public interface DeviceLoggingBuilder<T> {
    default DeviceLoggingBuilder<T> withName() {
        return this;
    }

    default DeviceLoggingBuilder<T> withPositionRotations() {
        return this;
    }

    default DeviceLoggingBuilder<T> withVelocityRotationsPerSec() {
        return this;
    }

    default DeviceLoggingBuilder<T> withAccelerationRotationsPerSecSq() {
        return this;
    }

    default DeviceLoggingBuilder<T> withMotorVoltage() {
        return this;
    }

    default DeviceLoggingBuilder<T> withMotorAmps() {
        return this;
    }

    default DeviceLoggingBuilder<T> withPGain() {
        return this;
    }

    default DeviceLoggingBuilder<T> withIGain() {
        return this;
    }

    default DeviceLoggingBuilder<T> withDGain() {
        return this;
    }

    default DeviceLoggingBuilder<T> withFeedForward() {
        return this;
    }

    default DeviceLoggingBuilder<T> withError() {
        return this;
    }

    default DeviceLoggingBuilder<T> withPIDOutput() {
        return this;
    }

    default DeviceLoggingBuilder<T> withMotorTemperature() {
        return this;
    }

    default DeviceLoggingBuilder<T> withPID() {
        return withPGain().withDGain().withIGain().withFeedForward().withError().withPIDOutput();
    }

    DeviceLogger build();
}
