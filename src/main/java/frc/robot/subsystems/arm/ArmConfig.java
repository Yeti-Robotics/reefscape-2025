package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

class ArmConfig {
    static final int ARM_KRAKEN_ID = 21;
    static final int ARM_CANCODER_ID = 5;

    static final TalonFXConfiguration armMotorConfig =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withSensorToMechanismRatio(0)
                                    .withRotorToSensorRatio(113)
                                    .withFeedbackRemoteSensorID(ARM_CANCODER_ID)
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(
                            new Slot0Configs()
                                    .withKP(540)
                                    .withKI(0)
                                    .withKD(200)
                                    .withKG(13.2)
                                    .withKA(1)
                                    .withKV(.75)
                                    .withGravityType(GravityTypeValue.Arm_Cosine));

    static final CANcoderConfiguration armCanCoderConfig =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withMagnetOffset(0)
                                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
}
