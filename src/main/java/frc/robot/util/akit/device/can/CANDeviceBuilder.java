package frc.robot.util.akit.device.can;

import frc.robot.util.akit.device.ConfigurableDeviceBuilder;
import frc.robot.util.akit.device.DeviceInputs;

public abstract class CANDeviceBuilder<
                D, C, I extends DeviceInputs, U extends ConfigurableDeviceBuilder<D, C, I, U>>
        extends ConfigurableDeviceBuilder<D, C, I, U> {
    public static double CONNECTED_DEBOUNCE_TIME = 0.5;

    protected CANDeviceBuilder(D device) {
        super(device);
    }

    // TODO: add methods to specify retry/timeout for CAN config sync
    public abstract U syncConfigs();
}
