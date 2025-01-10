package frc.robot;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final TalonFX claw;

    public static class IntakeConstants {
        public static final int clawId = 9;
        public static final InvertedValue clawInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue clawNeutralMode = NeutralModeValue.Coast;
        public static final double positionStatusFrame = 0.05;
        public static final double velocityStatusFrame = 0.01;
    }

    // Constructor that sets configs for claw
    public CoralIntake() {
        claw = new TalonFX(IntakeConstants.clawId, RIO_BUS);

        var clawConfigurator = claw.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = IntakeConstants.clawInversion;
        configs.MotorOutput.NeutralMode = IntakeConstants.clawNeutralMode;
        clawConfigurator.apply(configs);
    }

    private void setClawSpeed(double speed) {
        claw.set(speed);
    }

    private void stopClaw() {
        claw.stopMotor();
    }

    // Spins claw at the set speed, can be both in and out depending on speed
    public Command spinClaw(double speed) {
        return startEnd(() -> setClawSpeed(speed), this::stopClaw);
    }
}
