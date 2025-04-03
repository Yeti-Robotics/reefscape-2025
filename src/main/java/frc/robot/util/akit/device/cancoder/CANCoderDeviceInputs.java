package frc.robot.util.akit.device.cancoder;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.akit.device.PhysicalDeviceInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CANCoderDeviceInputs extends PhysicalDeviceInputs {
    public Angle position;
    public Angle absolutePosition;
}
