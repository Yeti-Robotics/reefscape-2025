package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

class AlgaeRollerConfigs {
    static final int ROLLER_ID = 8;
    static final int ROLLER_CANCODER_ID = 5;
    static final InvertedValue ROLLER_INVERSION = InvertedValue.Clockwise_Positive;
    static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast;
    static final double POSITION_STATUS_FRAME = 0.05;
    static final double VELOCITY_STATUS_FRAME = 0.01;
    static final double GEAR_RATIO = 0; // placeholder

    static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs()
                    .withKP(0) // placeholder alphabot
                    .withKI(0) // placeholder alphabot
                    .withKD(0) // placeholder alphabot
                    .withKG(0) // placeholder alphabot
                    .withKV(0) // placeholder alphabot
                    .withKA(0) // placeholder alphabot
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final TalonFXConfiguration configs =
            new TalonFXConfiguration()
                    .withFeedback(
                            new FeedbackConfigs()
                                    .withFeedbackRemoteSensorID(0)
                                    .withFeedbackSensorSource(
                                            FeedbackSensorSourceValue.FusedCANcoder)
                                    .withSensorToMechanismRatio(GEAR_RATIO) // alphabot
                                    .withRotorToSensorRatio(1)) // alphabot
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(SLOT_0_CONFIGS);
}
