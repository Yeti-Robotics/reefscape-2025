package frc.robot.util.akit.device.can.talon.inputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXPositionInputs {
    public Angle positionRotations;
    public AngularVelocity velocityRotationsPerSec;
    public AngularAcceleration accelerationRotationsPerSecSq;
}
