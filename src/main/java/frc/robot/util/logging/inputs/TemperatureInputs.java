package frc.robot.util.logging.inputs;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TemperatureInputs {
    public double processorTempCelsius;
    public double ancillaryTempCelsius;
    public double deviceTempCelsius;
}
