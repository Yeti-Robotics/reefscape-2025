package frc.robot.util.akit.device.talon;

import edu.wpi.first.units.measure.*;
import frc.robot.util.akit.device.PhysicalDeviceInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXDeviceInputs extends PhysicalDeviceInputs {
    // Electrical Inputs
    public Voltage motorVoltage;
    public Current motorAmps;

    // Motion Inputs
    public Angle positionRotations;
    public AngularVelocity velocityRotationsPerSec;
    public AngularAcceleration accelerationRotationsPerSecSq;

    // PID Controller Inputs
    public double pGain;
    public double iGain;
    public double dGain;
    public double feedForward;
    public double error;
    public double pidOutput;

    // Temperature Inputs
    public Temperature motorTemperature;
}
