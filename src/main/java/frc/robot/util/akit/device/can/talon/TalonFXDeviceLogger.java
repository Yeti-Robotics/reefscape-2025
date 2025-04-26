package frc.robot.util.akit.device.can.talon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANUtil;

public class TalonFXDeviceLogger implements DeviceLogger<TalonFXDeviceInputs> {
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
    private final Debouncer connectedDebouncer = new Debouncer(CANUtil.CONNECTED_DEBOUNCE_TIME);

    public TalonFXDeviceLogger(TalonFX talon) {
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

        BaseStatusSignal.setUpdateFrequencyForAll(
                CANUtil.TALON_DEFAULT_UPDATE_HZ,
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
    }

    public void updateInputs(TalonFXDeviceInputs inputs) {
        StatusCode refreshCode = BaseStatusSignal.refreshAll(
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
        inputs.motorInputs.motorVoltage = motorVoltage.getValue();
        inputs.motorInputs.motorAmps = motorAmps.getValue();
        inputs.positionInputs.positionRotations = positionRotations.getValue();
        inputs.positionInputs.velocityRotationsPerSec = velocityRotationsPerSec.getValue();
        inputs.positionInputs.accelerationRotationsPerSecSq = accelerationRotationsPerSecSq.getValue();

        inputs.pidInputs.pGain = pGain.getValue();
        inputs.pidInputs.iGain = iGain.getValue();
        inputs.pidInputs.dGain = dGain.getValue();
        inputs.pidInputs.feedForward = feedForward.getValue();
        inputs.pidInputs.error = error.getValue();
        inputs.pidInputs.pidOutput = pidOutput.getValue();

        inputs.motorInputs.motorTemperature = motorTemperature.getValue();
    }
}
