package frc.robot.util.logging.inputs;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.BridgeOutputValue;
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
