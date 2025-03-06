package frc.robot.util.logging.inputs;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {
    public double positionRotations;
    public double velocityRotationsPerSec;
    public double accelerationRotationsPerSecSq;
    public double motorVoltage;
    public double motorAmps;
}
