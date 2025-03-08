package frc.robot.util.akit.logging.device.inputs;

import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TemperatureInputs implements DeviceInputs {
    public Temperature motorTemperature;
}
