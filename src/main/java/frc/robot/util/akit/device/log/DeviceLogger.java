package frc.robot.util.akit.device.log;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.akit.device.inputs.CANCoderDeviceInputs;
import frc.robot.util.akit.device.inputs.DeviceInputs;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputs;
import frc.robot.util.akit.io.InputLoggingIO;

public interface DeviceLogger<T extends DeviceInputs> extends InputLoggingIO<T> {
    static DeviceLogger<TalonFXDeviceInputs> forDevice(TalonFX talon) {
        return new TalonFXDeviceLogger(talon);
    }

    static DeviceLogger<CANCoderDeviceInputs> forDevice(CANcoder cancoder) {
        return new CANCoderDeviceLogger(cancoder);
    }
}
