package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class WristConfigs {
    static final InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final int DEVICE_ID = 9;
}
