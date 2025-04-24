package frc.robot.util.akit.device.can.cancolor;

import frc.robot.util.akit.device.PhysicalDeviceInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CANColorInputs extends PhysicalDeviceInputs {
    public double temperatureCelsius;

    // RGB
    public double red;
    public double green;
    public double blue;

    public double promixity;
}
