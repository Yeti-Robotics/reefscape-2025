package frc.robot.subsystems.climber.io;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConfig {
    static final int CLIMBER_MOTOR_ID = 23;

    public static final double CLIMB_SPEED = 0.75;
    public static final double UNCLIMB_SPEED = -0.7;

    static final TalonFXConfiguration climberTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKA(0)
                    .withKV(0))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(0)
                    .withMotionMagicCruiseVelocity(0)
                    .withMotionMagicJerk(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                    new FeedbackConfigs()
                            .withRotorToSensorRatio(5.333333333333333) // def change that later
                            .withSensorToMechanismRatio(3.47222222222222) // def change that later
                    );
}
