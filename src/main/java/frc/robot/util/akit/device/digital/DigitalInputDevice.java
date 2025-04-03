package frc.robot.util.akit.device.digital;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.akit.device.DeviceBuilder;
import frc.robot.util.akit.device.inputs.DigitalInputDeviceInputsAutoLogged;
import frc.robot.util.akit.device.DeviceLogger;

public class DigitalInputDevice
        extends DeviceBuilder<
                DigitalInput, DigitalInputConfig, DigitalInputDeviceInputs, DigitalInputDevice> {
    private DigitalInputDevice(DigitalInput device) {
        super(device);
    }

    public static DigitalInputDevice configure(int deviceID) {
        return new DigitalInputDevice(new DigitalInput(deviceID));
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
    protected DeviceLogger<DigitalInputDeviceInputs> createLogger() {
        return new DigitalInputDeviceLogger(getDevice());
    }

    @Override
    protected DigitalInputConfig getDefaultConfig() {
        return new DigitalInputConfig();
    }

    @Override
    public DigitalInput getDevice() {
        if (config == null) {
            return super.getDevice();
        } else {
            return new CustomDigitalInput(super.getDevice().getChannel(), config);
        }
    }
}
