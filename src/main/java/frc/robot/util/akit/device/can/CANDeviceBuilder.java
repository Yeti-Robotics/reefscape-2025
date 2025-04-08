package frc.robot.util.akit.device.can;

import frc.robot.util.akit.device.DeviceBuilder;
import frc.robot.util.akit.device.DeviceInputs;

public abstract class CANDeviceBuilder<D, C, I extends DeviceInputs, U extends DeviceBuilder<D, C, I, U>> extends DeviceBuilder<D, C, I, U> {
    protected CANDeviceBuilder(D device) {
        super(device);
    }

    // TODO: add methods to specify retry/timeout for CAN config sync
    public abstract U syncConfigs();
}
