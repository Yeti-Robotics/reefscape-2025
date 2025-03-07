package frc.robot.util.akit.logging.device;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public interface DeviceLoggingBuilder<T> {
    default Supplier<Angle> withPositionRotations() { return Units.Rotations::zero; }
    default Supplier<AngularVelocity> withVelocityRotationsPerSec() { return Units.RotationsPerSecond::zero;}
    default Supplier<AngularAcceleration> withAccelerationRotationsPerSecSq() { return Units.RotationsPerSecondPerSecond::zero; }
    default Supplier<Voltage> withMotorVoltage() { return Units.Volts::zero; }
    default Supplier<Current> withMotorAmps() { return Units.Amps::zero; }

    default DoubleSupplier withPGain() { return () -> 0.0; }
    default DoubleSupplier withIGain() { return () -> 0.0; }
    default DoubleSupplier withDGain() { return () -> 0.0; }

    default DoubleSupplier withFeedForward() { return () -> 0.0; }
    default DoubleSupplier withError() { return () -> 0.0; }
    default DoubleSupplier withPidOutput() { return () -> 0.0; }

    default Supplier<Angle> withPositionReferenceRotations() { return Units.Rotations::zero; }
    default Supplier<AngularVelocity> withVelocityReferenceRotationsPerSec() { return Units.RotationsPerSecond::zero;}

    default Supplier<Temperature> withProcessorTempCelsius() { return Units.Celsius::zero; }
    default Supplier<Temperature> withAncillaryTempCelsius()  { return Units.Celsius::zero; }
    default Supplier<Temperature> withDeviceTempCelsius()  { return Units.Celsius::zero; }
}
