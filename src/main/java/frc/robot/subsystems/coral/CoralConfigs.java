package frc.robot.subsystems.coral;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class CoralConfigs {
        protected static final int CLAW_ID = 9;
        protected static final InvertedValue CLAW_INVERSION = InvertedValue.Clockwise_Positive;
        protected static final NeutralModeValue CLAW_NEUTRAL_MODE = NeutralModeValue.Coast;
        protected static final double POSITION_STATUS_FRAME = 0.05;
        protected static final double VELOCITY_STATUS_FRAME = 0.01;
}
