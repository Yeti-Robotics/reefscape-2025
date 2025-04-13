package frc.robot.subsystems.coral.grabber;

import static frc.robot.constants.Constants.RIO_BUS;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.state.StatefulSubsystem;

@Logged
public class GrabberSubsystem extends StatefulSubsystem<GrabberState> {
    private final TalonFX claw = new TalonFX(GrabberConfig.CLAW_ID, RIO_BUS);
    private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);
    public final Trigger hasCoralTrigger;
    public final Trigger doesNotHaveCoralTrigger;
    // private final CurrentLimitsConfigs limit = new CurrentLimitsConfigs()
    // .withStatorCurrentLimit().withSupplyCurrentLimit();

    private final Canandcolor clawSwitch = new Canandcolor(GrabberConfig.GRABBER_CANANDCOLOR);

    public GrabberSubsystem() {
        super(GrabberState.OFF);
        claw.getConfigurator().apply(GrabberConfig.coralMotorConfig);

        new Trigger(this::hasCoral).onTrue(transitionTo(GrabberState.OFF));
        new Trigger(() -> getCurrentState() == GrabberState.ROLL_OUT)
                .debounce(1)
                .onTrue(transitionTo(GrabberState.OFF));
        hasCoralTrigger = new Trigger(this::hasCoral);
        doesNotHaveCoralTrigger = new Trigger(this::doesNotHaveCoral);
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
        return clawSwitch.getProximity() < 0.2;
    }

    public boolean doesNotHaveCoral() {
        return clawSwitch.getProximity() > 0.32;
    }

    //    public boolean hasAlgae() { return clawSwitch.getProximity(); }
    //
    //    public boolean doesNotHaveAlgae() { return clawSwitch.getColor(); }
}
