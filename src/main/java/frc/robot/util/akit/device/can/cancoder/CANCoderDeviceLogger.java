package frc.robot.util.akit.device.can.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANConstants;

public class CANCoderDeviceLogger implements DeviceLogger<CANCoderDeviceInputs> {
    private final StatusSignal<Angle> position;
    private final StatusSignal<Angle> absolutePosition;
    private final Debouncer connectedDebouncer = new Debouncer(CANConstants.CONNECTED_DEBOUNCE_TIME);

    public CANCoderDeviceLogger(CANcoder cancoder) {
        position = cancoder.getPosition();
        absolutePosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(CANConstants.CANCODER_DEFAULT_UPDATE_HZ, position, absolutePosition);
    }

    public void updateInputs(CANCoderDeviceInputs inputs) {
        StatusCode refreshCode = BaseStatusSignal.refreshAll(position, absolutePosition);

        inputs.isConnected = connectedDebouncer.calculate(refreshCode.isOK());
        inputs.position = position.getValue();
        inputs.absolutePosition = absolutePosition.getValue();
    }
}
