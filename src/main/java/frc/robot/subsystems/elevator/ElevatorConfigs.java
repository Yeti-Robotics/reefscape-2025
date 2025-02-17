package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

class ElevatorConfigs {
    static final int primaryElevatorMotorID = 42;
    static final int secondaryElevatorMotorID = 15;
    static final int magSwitchID = 0; // placeholder

    static final Slot1Configs SLOT_1_SIM_CONFIGS =
            new Slot1Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(1)
                    .withGravityType(GravityTypeValue.Elevator_Static);

    static final TalonFXConfiguration talonFXConfigs =
            new TalonFXConfiguration()
                    .withSlot0(
                            new Slot0Configs()
                                    .withKP(2.4) // placeholder
                                    .withKI(0) // placeholder
                                    .withKD(0.1)) // placeholder
                    .withSlot1(SLOT_1_SIM_CONFIGS)
                    .withMotionMagic(
                            new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(1) // placeholder
                                    .withMotionMagicAcceleration(1) // placeholder
                                    .withMotionMagicJerk(1)); // placeholder
}
