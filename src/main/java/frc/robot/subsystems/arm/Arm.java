package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.robot.constants.Constants;

public class Arm {
    private final TalonFX armKraken;
    final MotionMagicVoltage magicRequest;

    public Arm() {
        armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.CANIVORE_BUS);
        CANcoder armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID, Constants.CANIVORE_BUS);

        var armConfigurator = armKraken.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackRemoteSensorID(0)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                        .withSensorToMechanismRatio(1)
                        .withRotorToSensorRatio(1))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(ArmConfig.ARM_INVERSION)
                        .withNeutralMode(ArmConfig.ARM_NEUTRAL_MODE));

        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Slot0 = ArmConfig.SLOT_0_CONFIGS;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = .1;
        motionMagicConfigs.MotionMagicAcceleration = .2;
        motionMagicConfigs.MotionMagicJerk = 1;

        magicRequest = new MotionMagicVoltage(0);

        armKraken.getRotorPosition().waitForUpdate(ArmConfig.ARM_POSITION_STATUS_FRAME);

        armConfigurator.apply(talonFXConfiguration);

        var armEncoderConfigurator = armEncoder.getConfigurator();
        var cancoderConfiguration = new CANcoderConfiguration();

        cancoderConfiguration.MagnetSensor.MagnetOffset = ArmConfig.MAGNET_OFFSET;
        cancoderConfiguration.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive;
        armEncoderConfigurator.apply(cancoderConfiguration);
    }

    public enum Position {
        LOW(30), //placeholder
        MID(60), //placeholder
        HIGH(90); //placeholder

        private final int value;

        Position(final int value){
            this.value = value;
        }

        public int getValue() {return value;}
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

    public void target(Position position) {
        armKraken.setControl(magicRequest.withPosition(position.getValue()));
    }
}
