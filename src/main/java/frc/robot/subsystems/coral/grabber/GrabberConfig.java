package frc.robot.subsystems.coral.grabber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class GrabberConfig {
    static final int CLAW_ID = 12;
    static final double OUTSPIT = -0.1;
    static final double INTAKE = 1;
    static final double ALGAE_INTAKE = 0.7;
    static final double ALL_IN = 1;
    static final double HOLD = 0.1;
    static final double ALGAE_HOLD = 0.6;
    static final double ALGAE_SHOOT = -1.0;
    static final TalonFXConfiguration coralMotorConfig =
            new TalonFXConfiguration()
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(new Slot0Configs().withKV(1).withKA(1))
                    .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(1));
    // .withCurrentLimits();

    static final int GRABBER_CANANDCOLOR = 0;
}
