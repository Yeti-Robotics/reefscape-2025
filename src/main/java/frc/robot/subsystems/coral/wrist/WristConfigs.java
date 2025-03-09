package frc.robot.subsystems.coral.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

class WristConfigs {
    static final double WRIST_TOLERANCE = 0.1;
    static final int WRIST_KRAKEN_ID = 19;
    static final int WRIST_CANCODER_ID = 30;
    static final double MAGNET_OFFSET = 0.501221;

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0)
                    .withKG(0);

    static final Slot1Configs SLOT_1_CONFIGS_SIM =
            new Slot1Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(32)
                    .withKS(0)
                    .withKV(0)
                    .withKA(1.8)
                    .withKG(0);

    static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs()
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicCruiseVelocity(0.5)
                    .withMotionMagicJerk(0);

    static final FeedbackConfigs FEEDBACK_CONFIGS =
            new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(3.75);

    static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS =
            new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake);

    static final TalonFXConfiguration wristMotorConfigs =
            new TalonFXConfiguration()
                    .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
                    .withSlot0(SLOT_0_CONFIGS)
                    .withSlot1(SLOT_1_CONFIGS_SIM)
                    .withMotionMagic(MOTION_MAGIC_CONFIGS)
                    .withFeedback(FEEDBACK_CONFIGS);

    static final CANcoderConfiguration wristEncoderConfigs =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                                    .withMagnetOffset(MAGNET_OFFSET)
                                    .withAbsoluteSensorDiscontinuityPoint(0.625));
}
