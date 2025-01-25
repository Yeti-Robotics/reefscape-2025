package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class ArmConfig {

    static final int ARM_KRAKEN_ID = 21;
    static final int ARM_CANCODER_ID = 5;

    static final InvertedValue ARM_INVERSION = InvertedValue.CounterClockwise_Positive;
    static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
    static final double ARM_POSITION_STATUS_FRAME = 0; // placeholder
    static final double ARM_VELOCITY_STATUS_FRAME = 0; // placeholder

    static final double ARM_P = 0; // placeholder
    static final double ARM_I = 0; // placeholder
    static final double ARM_D = 0; // placeholder
    static final double ARM_DEPLOY_LOWER_BOUND = 0; // placeholder

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(ARM_P)
                    .withKI(ARM_I)
                    .withKD(ARM_D)
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final double MAGNET_OFFSET = 0; // placeholder

    static final double GEAR_RATIO = 0; // placeholder
}
