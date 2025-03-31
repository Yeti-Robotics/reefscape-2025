package frc.robot.util.akit.device.log;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputs;

class TalonFXDeviceLogger implements DeviceLogger<TalonFXDeviceInputs> {
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorAmps;
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRotationsPerSec;
    private final StatusSignal<AngularAcceleration> accelerationRotationsPerSecSq;
    private final StatusSignal<Temperature> motorTemperature;
    private final StatusSignal<Double> pGain;
    private final StatusSignal<Double> iGain;
    private final StatusSignal<Double> dGain;
    private final StatusSignal<Double> feedForward;
    private final StatusSignal<Double> error;
    private final StatusSignal<Double> pidOutput;
    private final Debouncer connectedDebouncer = new Debouncer(CONNECTED_DEBOUNCE_TIME);

    TalonFXDeviceLogger(TalonFX talon) {
        motorVoltage = talon.getMotorVoltage();
        motorAmps = talon.getTorqueCurrent();
        positionRotations = talon.getPosition();
        velocityRotationsPerSec = talon.getVelocity();
        accelerationRotationsPerSecSq = talon.getAcceleration();
        motorTemperature = talon.getDeviceTemp();
        pGain = talon.getClosedLoopProportionalOutput();
        iGain = talon.getClosedLoopIntegratedOutput();
        dGain = talon.getClosedLoopDerivativeOutput();
        feedForward = talon.getClosedLoopFeedForward();
        error = talon.getClosedLoopError();
        pidOutput = talon.getClosedLoopOutput();
    }

    public void updateInputs(TalonFXDeviceInputs inputs) {
        StatusCode refreshCode =
                BaseStatusSignal.refreshAll(
                        motorVoltage,
                        motorAmps,
                        positionRotations,
                        velocityRotationsPerSec,
                        accelerationRotationsPerSecSq,
                        motorTemperature,
                        pGain,
                        iGain,
                        dGain,
                        feedForward,
                        error,
                        pidOutput);

        inputs.isConnected = connectedDebouncer.calculate(refreshCode.isOK());
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.motorAmps = motorAmps.getValue();
        inputs.positionRotations = positionRotations.getValue();
        inputs.velocityRotationsPerSec = velocityRotationsPerSec.getValue();
        inputs.accelerationRotationsPerSecSq = accelerationRotationsPerSecSq.getValue();

        inputs.pGain = pGain.getValue();
        inputs.iGain = iGain.getValue();
        inputs.dGain = dGain.getValue();
        inputs.feedForward = feedForward.getValue();
        inputs.error = error.getValue();
        inputs.pidOutput = pidOutput.getValue();

        inputs.motorTemperature = motorTemperature.getValue();
    }
}
