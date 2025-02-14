package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

class AlgaeArmConfigs {
    static final int ROLLER_ID = 8;
    static final int ROLLER_CANCODER_ID = 5;

    static final int ALGAE_ARM_KRAKEN_ID = 0; // placeholder
    static final int ALGAE_ARM_CANCODER_ID = 0; // placeholder
    static final double MAGNET_OFFSET = 0; // placeholder
    static final double GEAR_RATIO = 0; // placeholder

    static final double ARM_DEPLOY_LOWER_BOUND = 0; // placeholder //alphabot

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(0) // placeholder alphabot
                    .withKI(0) // placeholder alphabot
                    .withKD(0) // placeholder alphabot
                    .withKG(0) // placeholder alphabot
                    .withKV(0) // placeholder alphabot
                    .withKA(0) // placeholder alphabot
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final TalonFXConfiguration talonFXConfiguration =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(0) // placeholder
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder)
                                    .withSensorToMechanismRatio(GEAR_RATIO) // alphabot
                                    .withRotorToSensorRatio(0)) // placeholder alphabot
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(SLOT_0_CONFIGS);

    static final TalonFXConfiguration configs =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(0)
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder)
                                    .withSensorToMechanismRatio(GEAR_RATIO) // alphabot
                                    .withRotorToSensorRatio(1)) // alphabot
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(SLOT_0_CONFIGS);

    static final MotionMagicConfigs motionMagicConfigs =
            talonFXConfiguration
                    .MotionMagic
                    .withMotionMagicCruiseVelocity(0)
                    .withMotionMagicAcceleration(0)
                    .withMotionMagicJerk(0);

    static final CANcoderConfiguration cancoderConfiguration =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withSensorDirection(
                                            SensorDirectionValue.CounterClockwise_Positive)
                                    .withMagnetOffset(MAGNET_OFFSET));

    // static final InvertedValue ROLLER_INVERSION = InvertedValue.Clockwise_Positive;
    // static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast;
    static final double POSITION_STATUS_FRAME = 0.05;
    static final double VELOCITY_STATUS_FRAME = 0.01;
}
