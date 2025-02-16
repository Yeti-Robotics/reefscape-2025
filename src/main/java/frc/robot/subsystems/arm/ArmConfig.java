package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

class ArmConfig {

    static final int ARM_KRAKEN_ID = 47;
    static final int ARM_CANCODER_ID = 5;

    static final double MAGNET_OFFSET = 0; // placeholder
    static final double GEAR_RATIO = 113;

    static final double ARM_DEPLOY_LOWER_BOUND = 0; // placeholder //alphabot

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(540) // alphabot
                    .withKI(0) // alphabot
                    .withKD(200) // alphabot
                    .withKG(13.2) // alphabot
                    .withKV(1) // alphabot
                    .withKA(0.75) // alphabot
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final MotionMagicConfigs motionMagicConfigs =
            new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicAcceleration(2)
                    .withMotionMagicJerk(0);

    static final TalonFXConfiguration talonFXConfiguration =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(ARM_CANCODER_ID)
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder)
                                    .withSensorToMechanismRatio(GEAR_RATIO) // alphabot
                                    .withRotorToSensorRatio(1)) // alphabot
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(SLOT_0_CONFIGS)
                    .withMotionMagic(motionMagicConfigs);

    static final CANcoderConfiguration cancoderConfiguration =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withSensorDirection(
                                            SensorDirectionValue.CounterClockwise_Positive)
                                    .withMagnetOffset(MAGNET_OFFSET));
}
