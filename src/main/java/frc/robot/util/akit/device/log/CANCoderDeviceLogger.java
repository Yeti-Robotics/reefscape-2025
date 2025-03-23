package frc.robot.util.akit.device.log;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.akit.device.inputs.CANCoderDeviceInputs;

class CANCoderDeviceLogger implements DeviceLogger<CANCoderDeviceInputs> {
    private final StatusSignal<Angle> position;
    private final StatusSignal<Angle> absolutePosition;

    public CANCoderDeviceLogger(CANcoder cancoder) {
        position = cancoder.getPosition();
        absolutePosition = cancoder.getAbsolutePosition();
    }

    public void updateInputs(CANCoderDeviceInputs inputs) {
        StatusCode refreshCode = BaseStatusSignal.refreshAll(position, absolutePosition);

        inputs.isConnected = refreshCode.isOK();
        inputs.position = position.getValue();
        inputs.absolutePosition = absolutePosition.getValue();
    }
}
