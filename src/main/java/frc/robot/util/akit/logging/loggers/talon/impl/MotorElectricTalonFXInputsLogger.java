package frc.robot.util.akit.logging.loggers.talon.impl;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.akit.logging.device.inputs.MotorElectricInputs;
import frc.robot.util.akit.logging.loggers.talon.TalonFXDeviceInputsLogger;

public class MotorElectricTalonFXInputsLogger extends TalonFXDeviceInputsLogger<MotorElectricInputs> {
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public MotorElectricTalonFXInputsLogger(TalonFX talon) {
        super(talon);
        voltageSignal = talon.getMotorVoltage();
        currentSignal = talon.getTorqueCurrent();

        putStatusSignals(voltageSignal, currentSignal);
    }

    @Override
    public void updateInputs(MotorElectricInputs inputs) {
        inputs.motorAmps = currentSignal.getValue();
        inputs.motorVoltage = voltageSignal.getValue();
    }

    @Override
    protected Class<MotorElectricInputs> getDeviceInputsClass() {
        return MotorElectricInputs.class;
    }
}
