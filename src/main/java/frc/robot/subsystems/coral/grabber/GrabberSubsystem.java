package frc.robot.subsystems.coral.grabber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.state.StatefulSubsystem;

import static frc.robot.constants.Constants.RIO_BUS;

public class GrabberSubsystem extends StatefulSubsystem<GrabberState> {
    private final TalonFX claw = new TalonFX(GrabberConfig.CLAW_ID, RIO_BUS);
    // replace this with motion magic velo control
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);

    // idk if its DigitalInput or Cancolor, assuming the former for now
    private final DigitalInput clawSwitch = new DigitalInput(GrabberConfig.GRABBER_BEAM_BREAK);

    public GrabberSubsystem() {
        super(GrabberState.OFF);
        claw.getConfigurator().apply(GrabberConfig.coralMotorConfig);

        new Trigger(clawSwitch::get)
                .onChange(transitionTo(GrabberState.OFF));
    }

    @Override
    public StatusCode initializeTransition(GrabberState targetState) {
        return claw.setControl(dutyCycleReq.withOutput(targetState.getSpeed()));
    }

    // we assume that the transition to other states is (near) instantaneous
    @Override
    protected boolean isTransitionFinished() {
        return true;
    }

    public boolean hasCoral() {
        return clawSwitch.get();
    }
}
