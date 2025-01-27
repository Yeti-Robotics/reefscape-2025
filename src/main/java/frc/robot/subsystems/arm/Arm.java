package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class Arm {
    private final TalonFX armKraken;
    final MotionMagicVoltage magicRequest;

    public Arm() {
        armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, "canivoreBus");
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, "canivoreBus");

        var armConfigurator = armKraken.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        talonFXConfiguration.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.MotorOutput.Inverted = ArmConfig.ARM_INVERSION;
        talonFXConfiguration.MotorOutput.NeutralMode = ArmConfig.ARM_NEUTRAL_MODE;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 0;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 0;
        talonFXConfiguration.Slot0 = ArmConfig.SLOT_0_CONFIGS;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = .1;
        motionMagicConfigs.MotionMagicAcceleration = .2;
        motionMagicConfigs.MotionMagicJerk = 1;

        magicRequest = new MotionMagicVoltage(0);

        armKraken.getRotorVelocity().waitForUpdate(ArmConfig.ARM_VELOCITY_STATUS_FRAME);
        armKraken.getRotorPosition().waitForUpdate(ArmConfig.ARM_POSITION_STATUS_FRAME);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.MagnetOffset = ArmConfig.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive;
        armEncoderConfigurator.apply(cancoderConfiguration);
    }

    public void moveUp(double speed) {
        armKraken.set(Math.abs(speed));
    }

    private void moveDown(double speed) {
        armKraken.set(-Math.abs(speed));
    }

    public void stop() {
        armKraken.stopMotor();
    }

    public void target() {
        armKraken.setControl(magicRequest.withPosition(100));
    }
}
