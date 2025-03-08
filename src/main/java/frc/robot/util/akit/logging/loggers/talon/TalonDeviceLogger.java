package frc.robot.util.akit.logging.loggers.talon;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.akit.logging.device.inputs.DeviceInputs;

import java.util.HashMap;
import java.util.Map;

public class TalonDeviceLogger {
    private static final TalonFXDeviceInputsLogger<?>[] loggers = new
            TalonFXDeviceInputsLogger<?>[] {};

    public TalonDeviceLogger(TalonFX talonFX) {

    }

    public void update() {
        for (TalonFXDeviceInputsLogger<?> logger : loggers) {
            logger.getDeviceInputsClass()
        }
    }

    public <T extends DeviceInputs> TalonDeviceLogger with(T inputs) {
        if (inputs.getClass() == ) {}
    }

    public static TalonDeviceLogger logDevice(TalonFX talonFX) {
        return new TalonDeviceLogger(talonFX);
    }
}
