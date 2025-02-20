package frc.robot.subsystems.coral.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class ElevatorConfig {
    static final int primaryElevatorMotorID = 9;
    static final int secondaryElevatorMotorID = 11;
    static final int magSwitchID = 0; // placeholder
    static final double gearRatio = 44.0 / 18.0;

    private static final Slot0Configs SLOT_0_REAL_CONFIGS =
            new Slot0Configs()
                    .withKP(90)
                    .withKI(0)
                    .withKD(4.5)
                    .withKG(31.5)
                    .withKA(0.5)
                    .withKV(2)
                    .withGravityType(GravityTypeValue.Elevator_Static);

    private static final Slot1Configs SLOT_1_SIM_CONFIGS =
            new Slot1Configs()
                    .withKP(8)
                    .withKI(0)
                    .withKD(1)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0.1)
                    .withGravityType(GravityTypeValue.Elevator_Static);

    static final TalonFXConfiguration primaryTalonFXConfigs =
            new TalonFXConfiguration()
                    .withSlot0(SLOT_0_REAL_CONFIGS)
                    .withSlot1(SLOT_1_SIM_CONFIGS)
                    .withMotionMagic(
                            new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(7)
                                    .withMotionMagicAcceleration(14)
                                    .withMotionMagicJerk(0))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withRotorToSensorRatio(1.0)
                                    .withSensorToMechanismRatio(gearRatio));
    static final TalonFXConfiguration secondaryTalonFXConfigs =
            new TalonFXConfiguration()
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withRotorToSensorRatio(1.0)
                                    .withSensorToMechanismRatio(gearRatio));

    static final double HEIGHT_TOLERANCE = 0.05;
}
