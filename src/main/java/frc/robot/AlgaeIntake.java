package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final TalonFX roller;

    public static class IntakeConstants {
        public static final int rollerId = 8;
        public static final InvertedValue rollerInversion = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Coast;
        public static final double positionStatusFrame = 0.05;
        public static final double velocityStatusFrame = 0.01;
    }

    public AlgaeIntake() {
        roller = new TalonFX(IntakeConstants.rollerId, "rio");

        var rollerConfigurator = roller.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = IntakeConstants.rollerInversion;
        configs.MotorOutput.NeutralMode = IntakeConstants.rollerNeutralMode;
        roller.getRotorVelocity().waitForUpdate(IntakeConstants.velocityStatusFrame);
        roller.getRotorPosition().waitForUpdate(IntakeConstants.positionStatusFrame);
        rollerConfigurator.apply(configs);
    }

    private void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    private void stop() {
        roller.stopMotor();
    }

    public Command spinRoller(double speed) {
        return startEnd(() -> setRollerSpeed(speed), this::stop);
    }

}