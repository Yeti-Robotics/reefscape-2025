package frc.robot.util.akit.device.inputs;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CANCoderDeviceInputs extends PhysicalDeviceInputs {
    public Angle position;
    public Angle absolutePosition;
}
