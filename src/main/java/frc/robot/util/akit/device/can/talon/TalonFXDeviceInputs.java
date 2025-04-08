package frc.robot.util.akit.device.can.talon;

import edu.wpi.first.units.measure.*;
import frc.robot.util.akit.device.PhysicalDeviceInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXDeviceInputs extends PhysicalDeviceInputs {
    public Voltage motorVoltage;
    public Current motorAmps;

    public Angle positionRotations;
    public AngularVelocity velocityRotationsPerSec;
    public AngularAcceleration accelerationRotationsPerSecSq;

    public double pGain;
    public double iGain;
    public double dGain;
    public double feedForward;
    public double error;
    public double pidOutput;

    public Temperature motorTemperature;
}
