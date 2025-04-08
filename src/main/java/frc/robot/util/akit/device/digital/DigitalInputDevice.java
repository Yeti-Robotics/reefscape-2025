package frc.robot.util.akit.device.digital;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.akit.device.DeviceBuilder;
import frc.robot.util.akit.device.DeviceLogger;

public class DigitalInputDevice
        extends DeviceBuilder<
                DigitalInput, DigitalInputConfig, DigitalInputDeviceInputs, DigitalInputDevice> {
    private final int deviceID;

    private DigitalInputDevice(int id) {
        super(null);
        this.deviceID = id;
    }

    public static DigitalInputDevice configure(int deviceID) {
        return new DigitalInputDevice(deviceID);
    }

    public DigitalInputDevice withSimDevice(SimDevice simDevice) {
        getConfig().device = simDevice;
        return this;
    }

    public DigitalInputDevice debounce(double debounce) {
        getConfig().debounce = debounce;
        return this;
    }

    public DigitalInputDevice debounceType(Debouncer.DebounceType debounceType) {
        getConfig().debounceType = debounceType;
        return this;
    }

    public DigitalInputDevice invert() {
        getConfig().isInverted = true;
        return this;
    }

    @Override
    protected DigitalInputDevice getDeviceBuilderClass() {
        return this;
    }

    @Override
    protected DigitalInputDeviceInputs createDeviceInputs() {
        return new DigitalInputDeviceInputsAutoLogged();
    }

    @Override
    protected DeviceLogger<DigitalInputDeviceInputs> getLogger() {
        return new DigitalInputDeviceLogger(getDevice());
    }

    @Override
    protected DigitalInputConfig getDefaultConfig() {
        return new DigitalInputConfig();
    }

    @Override
    public DigitalInput getDevice() {
        if (hasConfig()) {
            return new CustomDigitalInput(deviceID, getConfig());
        }

        return new DigitalInput(deviceID);
    }
}
