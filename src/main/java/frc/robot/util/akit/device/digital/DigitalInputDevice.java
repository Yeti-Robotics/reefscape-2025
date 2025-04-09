package frc.robot.util.akit.device.digital;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.akit.device.DeviceBuilder;
import frc.robot.util.akit.device.DeviceLogger;

public class DigitalInputDevice
        extends DeviceBuilder<
                DigitalInput, Object, DigitalInputDeviceInputs, DigitalInputDevice> {

    private DigitalInputDevice(DigitalInput digitalInput) {
        super(digitalInput);
    }

    public static DigitalInputDevice configure(int deviceID) {
        return new DigitalInputDevice(new DigitalInput(deviceID));
    }

    public DigitalInputDevice withSimDevice(SimDevice simDevice) {
        getDevice().setSimDevice(simDevice);
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
    protected Object getDefaultConfig() {
        return null;
    }
}
