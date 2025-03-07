package frc.robot.util.akit.logging.inputs.impl;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PIDInputs {
    public double pGain;
    public double iGain;
    public double dGain;

    public double feedForward;

    public double error;
    public double pidOutput;
}
