package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

class AlgaeArmConfigs {
    static final int ALGAE_ARM_KRAKEN_ID = 0;
    static final int ALGAE_ARM_CANCODER_ID = 0;
    static final double MAGNET_OFFSET = 0;
    static final double GEAR_RATIO = 9.83;

    static final double ARM_DEPLOY_LOWER_BOUND = 0; // alphabot

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(0) // alphabot
                    .withKI(0) // alphabot
                    .withKD(0) // alphabot
                    .withKG(0) // alphabot
                    .withKV(0) // alphabot
                    .withKA(0) // alphabot
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final TalonFXConfiguration talonFXConfiguration =
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
