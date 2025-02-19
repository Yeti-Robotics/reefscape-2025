package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

class ArmConfig {

    static final int ARM_KRAKEN_ID = 10;
    static final int ARM_CANCODER_ID = 0;

    static final double MAGNET_OFFSET = 0.512451;
    static final double GEAR_RATIO = 75.6055;

    static final double ARM_DEPLOY_LOWER_BOUND = 0;

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(1440)
                    .withKI(0)
                    .withKD(300)
                    .withKG(7.5)
                    .withKV(1)
                    .withKA(2)
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final Slot1Configs SLOT_1_WOOD_CONFIGS =
            new Slot1Configs().withKP(256).withKI(8).withKG(1).withKD(8).withKA(3);

    static final MotionMagicConfigs motionMagicConfigs =
            new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.5)
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicJerk(0);

    static final TalonFXConfiguration talonFXConfiguration =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(0)
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder)
                                    .withSensorToMechanismRatio(1)
                                    .withRotorToSensorRatio(GEAR_RATIO))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(SLOT_0_CONFIGS)
                    .withMotionMagic(motionMagicConfigs);

    static final CANcoderConfiguration cancoderConfiguration =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withSensorDirection(
                                            SensorDirectionValue.CounterClockwise_Positive)
                                    .withMagnetOffset(MAGNET_OFFSET)
                                    .withAbsoluteSensorDiscontinuityPoint(0.625));
}
