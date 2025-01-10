package frc.robot;


import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.EnumMap;
import java.util.Map;

public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final CANcoder wristEncoder;
    public static final InvertedValue motorInversion = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public enum Positions {
        VERTICAL,
        HORIZONTAL
    }

    private final EnumMap<Positions, Integer> positionsEnumMap = new EnumMap<>(Map.ofEntries(
        Map.entry(Positions.VERTICAL, 90),
        Map.entry(Positions.HORIZONTAL, 0)
    ));

    public WristSubsystem() {
        wristMotor = new TalonFX(9, "rio");
        wristEncoder = new CANcoder(9);

        var wristConfigurator = wristMotor.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = motorInversion;
        configs.MotorOutput.NeutralMode = neutralMode;
        wristConfigurator.apply(configs);
    }

    private void spinWrist(double speed) { wristMotor.set(speed); }

    private void stopWrist() {
        wristMotor.stopMotor();
    }

    public Command moveWrist(double speed) {
        return startEnd(() -> spinWrist(speed), this::stopWrist);
    }

    public void setPosition(double targetPosition) {
        MotionMagicExpoVoltage motionMagicVoltage = new MotionMagicExpoVoltage(targetPosition);
        wristMotor.setControl(motionMagicVoltage);
    }

    public void resetEncoder() {
        wristEncoder.setPosition(0);
    }
}

