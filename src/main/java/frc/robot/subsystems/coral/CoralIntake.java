package frc.robot.subsystems.coral;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StateManager;

public class CoralIntake extends SubsystemBase {
    private final TalonFX claw = new TalonFX(CoralConfigs.CLAW_ID, RIO_BUS);

    public enum CoralState {
        ROLL_OUT,
        ROLL_IN,
        OFF
    }

    public StateManager<CoralState> coralIntakeState = new StateManager<CoralState>()
            .withDefaultState(CoralState.OFF);

    public CoralIntake() {
        claw.getConfigurator().apply(CoralConfigs.coralMotorConfig);
    }

    @Override
    public void periodic() {
        switch (coralIntakeState.getCurrentState()) {
            case ROLL_IN:
                setClawSpeed(CoralConfigs.FORWARD_SPEED);
                coralIntakeState.finishTransition();
                break;
            case ROLL_OUT:
                setClawSpeed(CoralConfigs.BACKWARD_SPEED);
                coralIntakeState.finishTransition();
                break;
            default:
                stopClaw();
        }
    }

    private void setClawSpeed(double speed) {
        claw.set(speed);
    }

    private void stopClaw() {
        claw.stopMotor();
    }

    public Command spinClawForward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_IN));
    }

    public Command spinClawBackward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_OUT));
    }
}
