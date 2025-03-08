package frc.robot.util.akit.logging.loggers.talon.impl;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.akit.logging.device.inputs.MotorInputs;
import frc.robot.util.akit.logging.loggers.talon.TalonFXDeviceInputsLogger;

public class MotorTalonFXInputsLogger extends TalonFXDeviceInputsLogger<MotorInputs> {
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;

    public MotorTalonFXInputsLogger(TalonFX talon) {
        super(talon);
        positionSignal = talon.getPosition();
        velocitySignal = talon.getVelocity();
        accelerationSignal = talon.getAcceleration();

        putStatusSignals(positionSignal, velocitySignal, accelerationSignal);
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        inputs.positionRotations = positionSignal.getValue();
        inputs.velocityRotationsPerSec = velocitySignal.getValue();
        inputs.accelerationRotationsPerSecSq = accelerationSignal.getValue();
    }

    @Override
    protected Class<MotorInputs> getDeviceInputsClass() {
        return MotorInputs.class;
    }
}
