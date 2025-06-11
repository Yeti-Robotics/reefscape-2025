package frc.robot.subsystems.coral.grabber.io.sensor;

import frc.robot.subsystems.coral.grabber.io.GrabberConfig;
import frc.robot.util.akit.device.digital.DigitalInputDevice;
import frc.robot.util.akit.device.digital.DigitalInputDeviceInputs;

class SimulatedCoralSensor implements CoralSensor {
    private final DigitalInputDeviceInputs simulatedSwitchInputs = DigitalInputDevice.configure(GrabberConfig.GRABBER_CANANDCOLOR % DigitalInputDevice.MAX_DIGITAL_ID)
            .log("Grabber/SimulatedColorSensor")
            .getDeviceInputs();

    @Override
    public boolean hasCoral() {
        return simulatedSwitchInputs.isTriggered;
    }
}
