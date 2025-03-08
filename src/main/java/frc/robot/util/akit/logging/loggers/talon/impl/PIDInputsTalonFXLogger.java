package frc.robot.util.akit.logging.loggers.talon.impl;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.akit.logging.device.inputs.PIDInputs;
import frc.robot.util.akit.logging.loggers.talon.TalonFXDeviceInputsLogger;
import frc.robot.util.sim.TalonFXLogger;

public class PIDInputsTalonFXLogger extends TalonFXDeviceInputsLogger<PIDInputs> {
    private final StatusSignal<Double> proportionalGainSignal;
    private final StatusSignal<Double> derivativeGainSignal;
    private final StatusSignal<Double> integralGainSignal;
    private final StatusSignal<Double> feedForwardSignal;
    private final StatusSignal<Double> errorSignal;
    private final StatusSignal<Double> pidOutputSignal;

    public PIDInputsTalonFXLogger(TalonFX talon) {
        super(talon);
        proportionalGainSignal = talon.getClosedLoopProportionalOutput();
        derivativeGainSignal = talon.getClosedLoopDerivativeOutput();
        integralGainSignal = talon.getClosedLoopIntegratedOutput();
        feedForwardSignal = talon.getClosedLoopFeedForward();
        errorSignal = talon.getClosedLoopError();
        pidOutputSignal = talon.getClosedLoopOutput();

        putStatusSignals(proportionalGainSignal, derivativeGainSignal, integralGainSignal, feedForwardSignal, errorSignal, pidOutputSignal);
    }

    @Override
    public void updateInputs(PIDInputs inputs) {
        inputs.pGain = proportionalGainSignal.getValue();
        inputs.dGain = derivativeGainSignal.getValue();
        inputs.iGain = integralGainSignal.getValue();
        inputs.error = errorSignal.getValue();
        inputs.feedForward = feedForwardSignal.getValue();
        inputs.pidOutput = pidOutputSignal.getValue();
    }

    @Override
    protected Class<PIDInputs> getDeviceInputsClass() {
        return PIDInputs.class;
    }
}
