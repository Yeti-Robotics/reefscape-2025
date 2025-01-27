package frc.robot.subsystems.coral;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StateManager;

public class CoralIntake extends SubsystemBase {
    private final TalonFX claw;

    public static final double FORWARD_SPEED = -0.5;
    public static final double BACKWARD_SPEED = 1.0;

    public enum CoralState {
        ROLL_OUT,
        ROLL_IN,
        OFF
    }

    public StateManager<CoralState> coralIntakeState = new StateManager<CoralState>()
            .withDefaultState(CoralState.OFF);

    public CoralIntake() {
        claw = new TalonFX(CoralConfigs.CLAW_ID, RIO_BUS);

        var clawConfigurator = claw.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = CoralConfigs.CLAW_INVERSION;
        configs.MotorOutput.NeutralMode = CoralConfigs.CLAW_NEUTRAL_MODE;
        clawConfigurator.apply(configs);
    }

    /*
    Arm - move up to a position to score coral
    Tray - detect coral
    CoralIntake - puts coral into claw

    So when we get a coral
           1. Check DIO
           2. Set wanted state to roll in
           3. Wait until the DIO in the tray no longer detects coral
           4. Once it is no longer detected, move arm/elevator
     */

    @Override
    public void periodic() {
        switch (coralIntakeState.getCurrentState()) {
            case ROLL_IN:
                setClawSpeed(FORWARD_SPEED);
                coralIntakeState.finishTransition();
                break;
            case ROLL_OUT:
                setClawSpeed(BACKWARD_SPEED);
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

    // Spins claw at the set speed, can be both in and out depending on speed
    public Command spinClawForward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_IN));
    }

    public Command spinClawBackward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_OUT));
    }
}
