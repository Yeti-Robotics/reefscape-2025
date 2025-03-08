package frc.robot.subsystems.coral.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.Robot;

class ArmConfig {

    static final int ARM_KRAKEN_ID = 10;
    static final int ARM_CANCODER_ID = 0;

    static final double MAGNET_OFFSET = -0.490479;
    static final double GEAR_RATIO = 75.6055;

    static final double ARM_DEPLOY_LOWER_BOUND = 0;

    private static final Slot0Configs SLOT_0_REAL_CONFIGS =
            new Slot0Configs()
                    .withKP(1100)
                    .withKI(0)
                    .withKD(350)
                    .withKG(10)
                    .withKV(1)
                    .withKA(2)
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final Slot1Configs SLOT_1_WOOD_CONFIGS =
            new Slot1Configs()
                    .withKP(256)
                    .withKI(8)
                    .withKG(1)
                    .withKD(8)
                    .withKA(3)
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    private static final Slot2Configs SLOT_2_SIM_CONFIGS =
            new Slot2Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(600)
                    .withKG(0)
                    .withKV(0)
                    .withKA(60)
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final MotionMagicConfigs motionMagicConfigs =
            new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(2)
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicJerk(0);

    static final TalonFXConfiguration talonFXConfiguration =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(ARM_CANCODER_ID)
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder)
                                    .withRotorToSensorRatio(Robot.isReal() ? GEAR_RATIO : 1)
                                    .withSensorToMechanismRatio(Robot.isReal() ? 1 : GEAR_RATIO))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(
                                            Robot.isReal()
                                                    ? InvertedValue.Clockwise_Positive
                                                    : InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(SLOT_0_REAL_CONFIGS)
                    .withSlot1(SLOT_1_WOOD_CONFIGS)
                    .withSlot2(SLOT_2_SIM_CONFIGS)
                    .withMotionMagic(motionMagicConfigs);

    static final CANcoderConfiguration cancoderConfiguration =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withSensorDirection(
                                            SensorDirectionValue.CounterClockwise_Positive)
                                    .withMagnetOffset(MAGNET_OFFSET)
                                    .withAbsoluteSensorDiscontinuityPoint(0.625));

    static final double ANGLE_TOLERANCE = 0.05;
}
