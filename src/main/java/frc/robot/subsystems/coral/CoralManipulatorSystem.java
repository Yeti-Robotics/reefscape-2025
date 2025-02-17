package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.util.state.StatefulSubsystem;

public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final GrabberSubsystem grabber = new GrabberSubsystem();

    public CoralManipulatorSystem() {
        super(CoralManipulatorState.IDLE);
    }

    @Override
    protected void runPeriodic() {
        if (grabber.hasCoral() && getCurrentState() != CoralManipulatorState.READY) {
            transitionToState(CoralManipulatorState.READY);
        }
    }

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand;

        switch (targetState) {
            case INTAKE, READY -> {
                coralManipulatorCommand = arm.transitionTo(targetState.getArmPosition())
                        .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                        .alongWith(grabber.transitionTo(targetState.getGrabberState()))
                        .handleInterrupt(this::failTransition);
            }
            default -> {
                coralManipulatorCommand = arm.transitionTo(targetState.getArmPosition())
                        .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                        .alongWith(grabber.transitionTo(targetState.getGrabberState()))
                        .handleInterrupt(this::failTransition);
            }
        }

        coralManipulatorCommand.schedule();

        return StatusCode.OK;
    }

    @Override
    protected boolean checkTransitionFinished() {
        return !arm.isTransitioning() && !elevator.isTransitioning() && !grabber.isTransitioning();
    }
}
