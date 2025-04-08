package frc.robot.util.akit.device.can.cancolor;

import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.akit.device.DeviceInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CANColorInputs implements DeviceInputs {
    public Temperature temperature;

    // RGB
    public double red;
    public double green;
    public double blue;

    // HSV
    public double hue;
    public double saturation;
    public double value;
}
