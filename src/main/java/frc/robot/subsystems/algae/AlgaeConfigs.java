package frc.robot.subsystems.algae;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class AlgaeConfigs {
        protected static final int ROLLER_ID = 8;
        protected static final InvertedValue ROLLER_INVERSION = InvertedValue.Clockwise_Positive;
        protected static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast;
        protected static final double POSITION_STATUS_FRAME = 0.05;
        protected static final double VELOCITY_STATUS_FRAME = 0.01;
}
