package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

class ClimberConfigs {
    static final int climberId = 12;
    static final InvertedValue climberInversion = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue climberNeutralMode = NeutralModeValue.Brake;
    static final int canCoderId = 3506;

    static final TalonFXConfiguration climberTalonFXConfigs =
            new TalonFXConfiguration()
                    .withSlot0(
                            new Slot0Configs()
                                    .withKP(0)
                                    .withKI(0)
                                    .withKD(0)
                                    .withKG(0)
                                    .withKA(0)
                                    .withKV(0))
                    .withMotionMagic(
                            new MotionMagicConfigs()
                                    .withMotionMagicAcceleration(0)
                                    .withMotionMagicCruiseVelocity(0)
                                    .withMotionMagicJerk(0))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(climberInversion)
                                    .withNeutralMode(climberNeutralMode))
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withRotorToSensorRatio(
                                            5.333333333333333) // def change that later
                                    .withSensorToMechanismRatio(
                                            3.47222222222222) // def change that later
                            );
    static final CANcoderConfiguration cancoderConfiguration =
            new CANcoderConfiguration()
                    .withMagnetSensor(
                            new MagnetSensorConfigs()
                                    .withMagnetOffset(0)
                                    .withSensorDirection(
                                            SensorDirectionValue.CounterClockwise_Positive));
}
