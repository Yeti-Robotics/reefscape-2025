package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class WristConfigs {
    static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    static final int deviceId = 9;
}
