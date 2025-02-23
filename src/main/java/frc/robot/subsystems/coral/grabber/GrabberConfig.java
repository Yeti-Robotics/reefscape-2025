package frc.robot.subsystems.coral.grabber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class GrabberConfig {
    static final int CLAW_ID = 12;
    static final double FORWARD_SPEED = -0.8;
    static final double BACKWARD_SPEED = 1;
    static final TalonFXConfiguration coralMotorConfig =
            new TalonFXConfiguration()
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(new Slot0Configs().withKV(1).withKA(1))
                    .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(1));

    static final int GRABBER_CANANDCOLOR = 0;
}
