package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.util.state.StatefulSubsystem;

@Logged
public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    @Logged(name = "Arm")
    public final ArmSubsystem arm = new ArmSubsystem();

    @Logged(name = "Elevator")
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    @Logged(name = "Grabber")
    public final GrabberSubsystem grabber = new GrabberSubsystem();

    public CoralManipulatorSystem() {
        super(CoralManipulatorState.IDLE);
    }

    @Logged(name = "States/Coral Manipulator State")
    public String getCoralManipulatorState() {
        return getCurrentState().toString();
    }

    @Logged(name = "States/Arm State")
    public String getArmState() {
        return arm.getCurrentState().toString();
    }

    @Logged(name = "States/Elevator State")
    public String getElevatorState() {
        return elevator.getCurrentState().toString();
    }

    @Logged(name = "States/Grabber State")
    public String getGrabberState() {
        return grabber.getCurrentState().toString();
    }

    @Logged(name = "States/Is arm transitioning?")
    public boolean isArmTransitioning() {
        return arm.isTransitioning();
    }

    @Logged(name = "States/Is elevator transitioning?")
    public boolean isElevatorTransitioning() {
        return elevator.isTransitioning();
    }

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand =
                arm.transitionTo(targetState.getArmPosition())
                        .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                        .alongWith(grabber.transitionTo(targetState.getGrabberState()));

        if (getCurrentState()
                .getElevatorPosition()
                .getHeight()
                .lt(ElevatorPosition.SAFE_POSITION.getHeight())) {
            coralManipulatorCommand =
                    elevator.transitionTo(ElevatorPosition.SAFE_POSITION)
                            .andThen(coralManipulatorCommand);
        }

        coralManipulatorCommand.schedule();

        return StatusCode.OK;
    }

    @Override
    protected boolean isTransitionFinished() {
        return !arm.isTransitioning() && !elevator.isTransitioning() && !grabber.isTransitioning();
    }
}
