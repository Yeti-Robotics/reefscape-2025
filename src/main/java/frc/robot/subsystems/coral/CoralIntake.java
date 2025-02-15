package frc.robot.subsystems.coral;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.StateManager;

public class CoralIntake extends SubsystemBase {
    private final TalonFX claw = new TalonFX(CoralConfigs.CLAW_ID, RIO_BUS);

    public enum CoralState {
        ROLL_OUT,
        ROLL_IN,
        OFF
    }

    private final StateManager<CoralState> coralIntakeState = new StateManager<CoralState>(CoralState.OFF);
    // replace this with motion magic velo control
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
    private final NeutralOut stopRequest = new NeutralOut();

    // idk if its DigitalInput or Cancolor, assuming the former for now
    private final DigitalInput clawSwitch = new DigitalInput(0);

    public CoralIntake() {
        claw.getConfigurator().apply(CoralConfigs.coralMotorConfig);

        new Trigger(clawSwitch::get)
                .onChange(runOnce(() -> {
                    coralIntakeState.transitionTo(CoralState.OFF);
                }));
    }

    @Override
    public void periodic() {
        if (coralIntakeState.isTransitioning()) {
            StatusCode code = switch (coralIntakeState.transitioningTo().orElse(CoralState.OFF)) {
                case ROLL_IN -> claw.setControl(dutyCycleReq.withOutput(CoralConfigs.FORWARD_SPEED));
                case ROLL_OUT -> claw.setControl(dutyCycleReq.withOutput(CoralConfigs.BACKWARD_SPEED));
                default -> claw.setControl(stopRequest);
            };

            if (code.isOK()) {
                coralIntakeState.finishTransition();
            } else {
                coralIntakeState.failTransition();
            }
        }
    }

    public Command spinClawForward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_IN));
    }

    public Command spinClawBackward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_OUT));
    }
}
