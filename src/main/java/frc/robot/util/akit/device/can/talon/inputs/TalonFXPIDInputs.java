package frc.robot.util.akit.device.can.talon.inputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TalonFXPIDInputs {
    public double pGain;
    public double iGain;
    public double dGain;
    public double feedForward;
    public double error;
    public double pidOutput;
}
