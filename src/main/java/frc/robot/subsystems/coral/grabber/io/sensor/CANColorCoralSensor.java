package frc.robot.subsystems.coral.grabber.io.sensor;

import frc.robot.subsystems.coral.grabber.io.GrabberConfig;
import frc.robot.util.akit.device.can.cancolor.CANColorDevice;
import frc.robot.util.akit.device.can.cancolor.CANColorInputs;

class CANColorCoralSensor implements CoralSensor {
    private static final double CORAL_PROXIMITY = 0.05;

    private final CANColorInputs canColorInputs = CANColorDevice.configure(GrabberConfig.GRABBER_CANANDCOLOR)
            .log("Grabber/ColorSensor")
            .getDeviceInputs();

    @Override
    public boolean hasCoral() {
        return canColorInputs.proximity < CORAL_PROXIMITY;
    }
}
