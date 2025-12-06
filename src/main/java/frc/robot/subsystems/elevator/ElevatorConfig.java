package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorConfig {
    static int primaryMotorID = 9;
    static int secondaryMotor = 11;
    // i believe ive gotta put a mag switch here
    // WEE WOO WEE WOO PUT A MAGSWITCH HERE DONT FORGET THIS THIS IS IMPORTATN

    private static final Slot0Configs SLOT_0_REAL_CONFIGS = new Slot0Configs()
            .withKP(256)
            .withKI(0)
            .withKD(32)
            .withKG(91.5)
            .withKV(7)
            .withKA(2)
            .withKS(14)
            .withGravityType(GravityTypeValue.Elevator_Static);

    private static final Slot1Configs SLOT_1_SIM_CONFIGS = new Slot1Configs()
            .withKP(24)
            .withKI(0)
            .withKD(24)
            .withKG(0)
            .withKV(0)
            .withKA(0.1)
            .withGravityType(GravityTypeValue.Elevator_Static);

    static final TalonFXConfiguration primaryTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_REAL_CONFIGS)
            .withSlot1(SLOT_1_SIM_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(15)
                    .withMotionMagicAcceleration(25)
                    .withMotionMagicJerk(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1.0).withSensorToMechanismRatio(44.0 / 18.0));
}
