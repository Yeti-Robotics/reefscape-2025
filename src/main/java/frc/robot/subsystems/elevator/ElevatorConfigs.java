package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

class ElevatorConfigs {
    static final int primaryElevatorMotorID = 42;
    static final int secondaryElevatorMotorID = 15;
    static final int magSwitchID = 0; // placeholder

    static final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    static {
        Slot0Configs slot0configs = new Slot0Configs()
                .withKP(2.4) //placeholder
                .withKI(0) //placeholder
                .withKD(0.1); //placeholder

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1; //placeholder
        motionMagicConfigs.MotionMagicAcceleration = 1; //placeholder
        motionMagicConfigs.MotionMagicJerk = 1; //placeholder

    }
}
