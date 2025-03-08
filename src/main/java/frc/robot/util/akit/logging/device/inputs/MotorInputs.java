package frc.robot.util.akit.logging.device.inputs;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs implements DeviceInputs {
    public Angle positionRotations;
    public AngularVelocity velocityRotationsPerSec;
    public AngularAcceleration accelerationRotationsPerSecSq;
}
