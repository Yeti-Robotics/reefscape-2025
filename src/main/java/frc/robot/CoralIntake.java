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

    private static class IntakeConfigs {
        public static final int CLAW_ID = 9;
        public static final InvertedValue CLAW_INVERSION = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue CLAW_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final double POSITION_STATUS_FRAME = 0.05;
        public static final double VELOCITY_STATUS_FRAME = 0.01;

    }
    public static final double FORWARD_SPEED = 1;
    public static final double BACKWARD_SPEED = -1;

    public CoralIntake() {
        claw = new TalonFX(IntakeConfigs.CLAW_ID, RIO_BUS);

        var clawConfigurator = claw.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = IntakeConfigs.CLAW_INVERSION;
        configs.MotorOutput.NeutralMode = IntakeConfigs.CLAW_NEUTRAL_MODE;
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

    public Command spinClawForward(){
        return spinClaw(FORWARD_SPEED);
    }

    public Command spinClawBackward(){
        return spinClaw(BACKWARD_SPEED);
    }
}
