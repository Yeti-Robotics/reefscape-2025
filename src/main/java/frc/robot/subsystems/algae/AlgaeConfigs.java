package frc.robot.subsystems.algae;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class AlgaeConfigs {
    static final int ROLLER_ID = 8;
    static final InvertedValue ROLLER_INVERSION = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast;
    static final double POSITION_STATUS_FRAME = 0.05;
    static final double VELOCITY_STATUS_FRAME = 0.01;
}
