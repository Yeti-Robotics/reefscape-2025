package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class ClimberConfigs {
    static final int MOTOR_ID = 12;
    static final InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
}
