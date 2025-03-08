package frc.robot.util.akit.logging.loggers.talon;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.akit.io.InputLoggingIO;
import frc.robot.util.akit.logging.device.inputs.DeviceInputs;

public abstract class TalonFXDeviceInputsLogger<T extends DeviceInputs> implements InputLoggingIO<T> {
    private StatusSignal<?>[] statusSignals;

    public TalonFXDeviceInputsLogger(TalonFX talon) {

    }

    public void putStatusSignals(StatusSignal<?>... statusSignals) {
        this.statusSignals = statusSignals;
    }

    public StatusSignal<?>[] getStatusSignals() {
        return statusSignals;
    }

    protected abstract Class<T> getDeviceInputsClass();
}
