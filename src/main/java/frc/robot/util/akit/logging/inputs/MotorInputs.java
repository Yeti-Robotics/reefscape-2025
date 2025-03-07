package frc.robot.util.akit.logging.inputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {
    public double positionRotations;
    public double velocityRotationsPerSec;
    public double accelerationRotationsPerSecSq;
    public double motorVoltage;
    public double motorAmps;
}
