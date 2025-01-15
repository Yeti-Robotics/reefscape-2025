package frc.robot.subsystems.coral;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class CoralConfigs {
    static final int CLAW_ID = 9;
    static final InvertedValue CLAW_INVERSION = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue CLAW_NEUTRAL_MODE = NeutralModeValue.Coast;
    static final double POSITION_STATUS_FRAME = 0.05;
    static final double VELOCITY_STATUS_FRAME = 0.01;
}
