package frc.robot.util.akit.logging.loggers.talon.impl;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.util.akit.logging.device.inputs.DeviceInputs;
import frc.robot.util.akit.logging.device.inputs.TemperatureInputs;
import frc.robot.util.akit.logging.loggers.talon.TalonFXDeviceInputsLogger;

public class TemperatureInputsTalonFXInputsLogger extends TalonFXDeviceInputsLogger<TemperatureInputs> {
    private final StatusSignal<Temperature> temperatureSignal;

    public TemperatureInputsTalonFXInputsLogger(TalonFX talon) {
        super(talon);
        temperatureSignal = talon.getDeviceTemp();
        putStatusSignals(temperatureSignal);
    }

    @Override
    public void updateInputs(TemperatureInputs inputs) {
        inputs.motorTemperature = temperatureSignal.getValue();
    }

    @Override
    protected Class<TemperatureInputs> getDeviceInputsClass() {
        return TemperatureInputs.class;
    }
}
