package frc.robot.util.akit.device.can.cancoder;

import frc.robot.util.akit.device.PhysicalDeviceInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CANCoderDeviceInputs extends PhysicalDeviceInputs {
    public double position;
    public double absolutePosition;
}
