package frc.robot.util.akit.device.can.talon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class TalonFXDeviceLogger implements DeviceLogger<TalonFXDeviceInputs> {
    private StatusSignal<Angle> positionRotations;
    private StatusSignal<AngularVelocity> velocityRotationsPerSec;
    private StatusSignal<AngularAcceleration> accelerationRotationsPerSecSq;

    private StatusSignal<Temperature> motorTemperature;
    private StatusSignal<Double> dutyCycleOutput;
    private StatusSignal<Voltage> motorVoltage;
    private StatusSignal<Current> motorAmps;

    private StatusSignal<Double> pGain;
    private StatusSignal<Double> iGain;
    private StatusSignal<Double> dGain;
    private StatusSignal<Double> feedForward;
    private StatusSignal<Double> error;
    private StatusSignal<Double> pidOutput;

    private final Debouncer connectedDebouncer = new Debouncer(CANUtil.CONNECTED_DEBOUNCE_TIME);

    private final StatusSignal<?>[] loggedStatusSignals;

    private boolean logPosition = false;
    private boolean logPID = false;
    private boolean logMotorInputs = false;

    public TalonFXDeviceLogger(TalonFX talon, Set<TalonFXDevice.TalonFXLogging> loggingTypes) {
        List<StatusSignal<?>> loggedStatusSignalsList = new ArrayList<>();

        if (loggingTypes.contains(TalonFXDevice.TalonFXLogging.POSITION)) {
            positionRotations = talon.getPosition();
            velocityRotationsPerSec = talon.getVelocity();
            accelerationRotationsPerSecSq = talon.getAcceleration();

            loggedStatusSignalsList.add(positionRotations);
            loggedStatusSignalsList.add(velocityRotationsPerSec);
            loggedStatusSignalsList.add(accelerationRotationsPerSecSq);
            logPosition = true;
        }

        if (loggingTypes.contains(TalonFXDevice.TalonFXLogging.MOTOR)) {
            motorTemperature = talon.getDeviceTemp();
            dutyCycleOutput = talon.getDutyCycle();
            motorVoltage = talon.getMotorVoltage();
            motorAmps = talon.getTorqueCurrent();

            loggedStatusSignalsList.add(motorTemperature);
            loggedStatusSignalsList.add(dutyCycleOutput);
            loggedStatusSignalsList.add(motorVoltage);
            loggedStatusSignalsList.add(motorAmps);
            logMotorInputs = true;
        }


        if (loggingTypes.contains(TalonFXDevice.TalonFXLogging.PID)) {
            pGain = talon.getClosedLoopProportionalOutput();
            iGain = talon.getClosedLoopIntegratedOutput();
            dGain = talon.getClosedLoopDerivativeOutput();
            feedForward = talon.getClosedLoopFeedForward();
            error = talon.getClosedLoopError();
            pidOutput = talon.getClosedLoopOutput();

            loggedStatusSignalsList.add(pGain);
            loggedStatusSignalsList.add(iGain);
            loggedStatusSignalsList.add(dGain);
            loggedStatusSignalsList.add(feedForward);
            loggedStatusSignalsList.add(error);
            loggedStatusSignalsList.add(pidOutput);
            logPID = true;
        }

        loggedStatusSignals = loggedStatusSignalsList.toArray(StatusSignal[]::new);

        BaseStatusSignal.setUpdateFrequencyForAll(CANUtil.TALON_DEFAULT_UPDATE_HZ, loggedStatusSignals);
    }

    public void updateInputs(TalonFXDeviceInputs inputs) {
        StatusCode refreshCode = BaseStatusSignal.refreshAll(loggedStatusSignals);

        inputs.isConnected = connectedDebouncer.calculate(refreshCode.isOK());

        if (logPosition) {
            inputs.positionInputs.positionRotations = positionRotations.getValueAsDouble();
            inputs.positionInputs.velocityRotationsPerSec = velocityRotationsPerSec.getValueAsDouble();
            inputs.positionInputs.accelerationRotationsPerSecSq = accelerationRotationsPerSecSq.getValueAsDouble();
        }

        if (logPID) {
            inputs.pidInputs.pGain = pGain.getValue();
            inputs.pidInputs.iGain = iGain.getValue();
            inputs.pidInputs.dGain = dGain.getValue();
            inputs.pidInputs.feedForward = feedForward.getValue();
            inputs.pidInputs.error = error.getValue();
            inputs.pidInputs.pidOutput = pidOutput.getValue();
        }

        if (logMotorInputs) {
            inputs.motorInputs.motorTemperature = motorTemperature.getValue();
            inputs.motorInputs.dutyCycle = dutyCycleOutput.getValue();
            inputs.motorInputs.motorVoltage = motorVoltage.getValue();
            inputs.motorInputs.motorAmps = motorAmps.getValue();
        }
    }
}
