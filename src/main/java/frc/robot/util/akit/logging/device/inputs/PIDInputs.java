package frc.robot.util.akit.logging.device.inputs;


import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PIDInputs implements DeviceInputs {
    public double pGain;
    public double iGain;
    public double dGain;
    public double feedForward;

    public double error;
    public double pidOutput;
}
