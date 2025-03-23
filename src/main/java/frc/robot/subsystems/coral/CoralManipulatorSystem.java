package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.arm.io.ArmIOTalonFX;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.util.state.StatefulSubsystem;
import java.util.Map;

public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {

    public final ArmSubsystem arm = new ArmSubsystem(new ArmIOTalonFX());

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public final GrabberSubsystem grabber = new GrabberSubsystem();

    public CoralManipulatorSystem() {
        super(CoralManipulatorState.L1);
    }

    private CoralManipulatorState queuedState = CoralManipulatorState.L1;

    public String getCQueuedState() {
        return queuedState.toString();
    }

    private CoralManipulatorState getQueuedState() {
        return queuedState;
    }

    public String getCoralManipulatorState() {
        return getCurrentState().toString();
    }

    public String getCoralManipulatorTransState() {
        return transitioningTo().orElse(CoralManipulatorState.L1).toString();
    }

    public String getArmState() {
        return arm.toString();
    }

    public String getElevatorState() {
        return elevator.getCurrentState().toString();
    }

    public String getGrabberState() {
        return grabber.getCurrentState().toString();
    }

    public boolean isElevatorTransitioning() {
        return elevator.isTransitioning();
    }

    public String elevatorTransitionState() {
        return elevator.transitioningTo().orElse(ElevatorPosition.HOLD).toString();
    }

    public void queueState(CoralManipulatorState state) {
        queuedState = state;
    }

    public Command selectQueuedStateCommand() {
        return new SelectCommand(
                Map.of(
                        CoralManipulatorState.L1, transitionTo(CoralManipulatorState.L1),
                        CoralManipulatorState.L2, transitionTo(CoralManipulatorState.L2),
                        CoralManipulatorState.L3, transitionTo(CoralManipulatorState.L3),
                        CoralManipulatorState.L4, transitionTo(CoralManipulatorState.L4)),
                this::getQueuedState);
    }

    public Command scoreState() {
        return new SelectCommand(
                Map.of(
                        CoralManipulatorState.L1, transitionTo(CoralManipulatorState.SCORE_L1),
                        CoralManipulatorState.L2, transitionTo(CoralManipulatorState.SCORE_L2),
                        CoralManipulatorState.L3, transitionTo(CoralManipulatorState.SCORE_L3),
                        CoralManipulatorState.L4, transitionTo(CoralManipulatorState.SCORE_L4)),
                this::getQueuedState);
    }

    public Command setQueueState(CoralManipulatorState queuedState) {
        return runOnce(() -> queueState(queuedState));
    }

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand;

        if (getCurrentState()
                        .getElevatorPosition()
                        .getHeight()
                        .lt(ElevatorPosition.SAFE_POSITION.getHeight())
                && getCurrentState() != targetState) {
            if (targetState == CoralManipulatorState.SCORE_L2) {
                coralManipulatorCommand =
                        elevator.transitionTo(targetState.getElevatorPosition())
                                .andThen(arm.transitionTo(targetState.getArmPosition()))
                                .andThen(grabber.transitionTo(targetState.getGrabberState()));
            } else {
                coralManipulatorCommand =
                        elevator.transitionTo(ElevatorPosition.SAFE_POSITION)
                                .andThen(arm.transitionTo(targetState.getArmPosition()))
                                .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                                .andThen(grabber.transitionTo(targetState.getGrabberState()));
            }
        } else {
            coralManipulatorCommand =
                    arm.transitionTo(targetState.getArmPosition())
                            .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                            .andThen(grabber.transitionTo(targetState.getGrabberState()));
        }

        coralManipulatorCommand.schedule();

        return StatusCode.OK;
    }

    @Override
    protected boolean isTransitionFinished() {
        return !elevator.isTransitioning() && !grabber.isTransitioning();
    }
}
