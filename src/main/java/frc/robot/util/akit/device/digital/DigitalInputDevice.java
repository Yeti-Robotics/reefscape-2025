package frc.robot.util.akit.device.digital;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.akit.device.DeviceBuilder;
import frc.robot.util.akit.device.DeviceLogger;

public class DigitalInputDevice extends DeviceBuilder<DigitalInput, DigitalInputDeviceInputs, DigitalInputDevice> {
    public static final int MAX_DIGITAL_ID = 31;

    private DigitalInputDevice(DigitalInput digitalInput) {
        super(digitalInput);
    }

    public static DigitalInputDevice configure(int deviceID) {
        return from(new DigitalInput(deviceID));
    }

    public static DigitalInputDevice from(DigitalInput digitalInput) {
        return new DigitalInputDevice(digitalInput);
    }

    public Trigger toTrigger() {
        return new Trigger(device::get);
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
        return new DigitalInputDeviceLogger(device);
    }
}
