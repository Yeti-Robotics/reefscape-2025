package frc.robot.util.akit.device.digital;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.akit.device.DeviceBuilder;
import frc.robot.util.akit.device.DeviceLogger;

public class DigitalInputDevice extends DeviceBuilder<DigitalInput, DigitalInputDeviceInputs, DigitalInputDevice> {

    private DigitalInputDevice(DigitalInput digitalInput) {
        super(digitalInput);
    }

    public static DigitalInputDevice configure(int deviceID) {
        return from(new DigitalInput(deviceID));
    }

    public static DigitalInputDevice from(DigitalInput digitalInput) {
        return new DigitalInputDevice(digitalInput);
    }

    public DigitalInputDevice usingSimDevice(SimDevice simDevice) {
        getDevice().setSimDevice(simDevice);
        return this;
    }

    public Trigger toTrigger() {
        return new Trigger(getDevice()::get);
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
}
