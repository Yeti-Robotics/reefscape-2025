package frc.robot.subsystems.coral.elevator.io;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

class ElevatorConfig {
    static final int primaryElevatorMotorID = 9;
    static final int secondaryElevatorMotorID = 11;
    static final int magSwitchID = 6;
    static final double gearRatio = 44.0 / 18.0;

    private static final Slot0Configs SLOT_0_REAL_CONFIGS =
            new Slot0Configs()
                    .withKP(256)
                    .withKI(0)
                    .withKD(32)
                    .withKG(50)
                    .withKV(1)
                    .withKA(.5)
                    .withKS(14)
                    .withGravityType(GravityTypeValue.Elevator_Static);
    private static final Slot1Configs SLOT_1_SIM_CONFIGS =
            new Slot1Configs()
                    .withKP(24)
                    .withKI(0)
                    .withKD(24)
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

    static final Angle HEIGHT_TOLERANCE = Units.Rotations.of(0.08);
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0.01;
}
