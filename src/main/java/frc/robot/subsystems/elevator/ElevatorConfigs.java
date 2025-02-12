package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

class ElevatorConfigs {
    static final int primaryElevatorMotorID = 9;
    static final int secondaryElevatorMotorID = 11;
    static final int magSwitchID = 0; // placeholder

    static final TalonFXConfiguration talonFXConfigs =
            new TalonFXConfiguration()
                    .withSlot0(
                            new Slot0Configs()
                                    .withKP(0) // placeholder
                                    .withKI(0) // placeholder
                                    .withKD(0)) // placeholder
                    .withMotionMagic(
                            new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(1) // placeholder
                                    .withMotionMagicAcceleration(1) // placeholder
                                    .withMotionMagicJerk(1)); // placeholder
}
