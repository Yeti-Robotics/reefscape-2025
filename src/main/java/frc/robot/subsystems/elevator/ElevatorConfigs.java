package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class ElevatorConfigs {
    static final int primaryElevatorMotorID = 9;
    static final int secondaryElevatorMotorID = 11;
    static final int magSwitchID = 0; // placeholder
    static final double gearRatio = 44.0 / 18.0;

    static final TalonFXConfiguration primaryTalonFXConfigs =
            new TalonFXConfiguration()
                    .withSlot0(
                            new Slot0Configs()
                                    .withKP(0) // placeholder
                                    .withKI(0) // placeholder
                                    .withKD(0)
                                    .withKG(31.5)
                                    .withKA(0)
                                    .withKV(0)) // placeholder
                    .withMotionMagic(
                            new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(1) // placeholder
                                    .withMotionMagicAcceleration(1) // placeholder
                                    .withMotionMagicJerk(1))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withRotorToSensorRatio(1.0)
                                    .withSensorToMechanismRatio(gearRatio)); // placeholder
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
}
