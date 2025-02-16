package frc.robot.subsystems.coral.grabber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.state.StatefulSubsystem;

import static frc.robot.constants.Constants.RIO_BUS;

public class GrabberSubsystem extends StatefulSubsystem<GrabberSubsystem.CoralState> {
    private final TalonFX claw = new TalonFX(GrabberConfig.CLAW_ID, RIO_BUS);

    public enum CoralState {
        ROLL_OUT(GrabberConfig.FORWARD_SPEED),
        ROLL_IN(GrabberConfig.BACKWARD_SPEED),
        OFF(0);

        private final double speed;

        CoralState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    // replace this with motion magic velo control
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
    private final NeutralOut stopRequest = new NeutralOut();

    // idk if its DigitalInput or Cancolor, assuming the former for now
    private final DigitalInput clawSwitch = new DigitalInput(0);

    public GrabberSubsystem() {
        super(CoralState.OFF);
        claw.getConfigurator().apply(GrabberConfig.coralMotorConfig);

        new Trigger(clawSwitch::get)
                .onChange(transitionTo(CoralState.OFF));
    }


    @Override
    public StatusCode initializeTransition(CoralState targetState) {
        return claw.setControl(dutyCycleReq.withOutput(targetState.getSpeed()));
    }

    @Override
    public boolean checkTransitionFinished() {
        return true;
    }
}
