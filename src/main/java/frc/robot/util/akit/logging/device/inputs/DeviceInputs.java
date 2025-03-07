package frc.robot.util.akit.logging.device.inputs;

import edu.wpi.first.units.measure.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DeviceInputs {
    public Angle positionRotations;
    public AngularVelocity velocityRotationsPerSec;
    public AngularAcceleration accelerationRotationsPerSecSq;

    public Voltage motorVoltage;
    public Current motorAmps;

    public double pGain;
    public double iGain;
    public double dGain;
    public double feedForward;

    public double error;
    public double pidOutput;

    public Temperature motorTemperature;
    public Supplier<Double> motorTemperatureSupplier;
}
