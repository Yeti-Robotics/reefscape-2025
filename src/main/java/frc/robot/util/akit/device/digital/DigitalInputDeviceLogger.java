package frc.robot.util.akit.device.digital;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.akit.device.DeviceLogger;

public class DigitalInputDeviceLogger implements DeviceLogger<DigitalInputDeviceInputs> {
    private final DigitalInput input;

    public DigitalInputDeviceLogger(DigitalInput input) {
        this.input = input;
    }

    @Override
    public void updateInputs(DigitalInputDeviceInputs inputs) {
        inputs.isTriggered = input.get();
    }
}
