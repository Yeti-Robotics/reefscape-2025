package frc.robot.util.akit.logging.device.inputs;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorElectricInputs implements DeviceInputs {
    public Voltage motorVoltage;
    public Current motorAmps;
}
