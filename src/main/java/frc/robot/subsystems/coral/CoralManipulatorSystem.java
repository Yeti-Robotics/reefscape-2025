package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.subsystems.coral.wrist.WristSubsystem;
import frc.robot.util.state.StatefulSubsystem;
import java.util.Map;

@Logged
public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    @Logged(name = "Arm")
    public final ArmSubsystem arm = new ArmSubsystem();

    @Logged(name = "Elevator")
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    @Logged(name = "Grabber")
    public final GrabberSubsystem grabber = new GrabberSubsystem();

    @Logged(name = "Wrist")
    public final WristSubsystem wrist = new WristSubsystem();

    public CoralManipulatorSystem() {
        super(CoralManipulatorState.IDLE);
    }

    private CoralManipulatorState queuedState = CoralManipulatorState.IDLE;

    @Logged(name = "States/Queued State")
    public String getCQueuedState() {
        return queuedState.toString();
    }

    private CoralManipulatorState getQueuedState() {
        return queuedState;
    }

    @Logged(name = "States/Coral Manipulator State")
    public String getCoralManipulatorState() {
        return getCurrentState().toString();
    }

    @Logged(name = "States/Coral Manipulator Transitioning State")
    public String getCoralManipulatorTransState() {
        return transitioningTo().orElse(CoralManipulatorState.IDLE).toString();
    }

    @Logged(name = "States/Wrist State")
    public String getWristState() {
        return wrist.getCurrentState().toString();
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

    @Logged(name = "States/Arm transitioning State")
    public String armTransitionState() {
        return arm.transitioningTo().orElse(ArmPosition.HOLD).toString();
    }

    @Logged(name = "States/Elevator transitioning State")
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
            if (targetState == CoralManipulatorState.L2
                    && arm.getCurrentState() != ArmPosition.DOWN) {
                coralManipulatorCommand =
                        arm.transitionTo(targetState.getArmPosition())
                                .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                                .andThen(wrist.transitionTo(targetState.getWristPosition()))
                                .andThen(grabber.transitionTo(targetState.getGrabberState()));
            } else if (arm.getCurrentState().getAngle().lt(ArmPosition.AWAY.getAngle())
                    || targetState.getArmPosition().getAngle().lt(ArmPosition.AWAY.getAngle())) {
                // safety stuff
                coralManipulatorCommand =
                        elevator.transitionTo(ElevatorPosition.SAFE_POSITION)
                                .andThen(wrist.transitionTo(targetState.getWristPosition()))
                                .andThen(arm.transitionTo(targetState.getArmPosition()))
                                .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                                .andThen(grabber.transitionTo(targetState.getGrabberState()));

            } else {
                // not safety stuff
                coralManipulatorCommand =
                        arm.transitionTo(targetState.getArmPosition())
                                .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                                .andThen(wrist.transitionTo(targetState.getWristPosition()))
                                .andThen(grabber.transitionTo(targetState.getGrabberState()));
            }
        } else {
            coralManipulatorCommand =
                    wrist.transitionTo(targetState.getWristPosition())
                            .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                            .alongWith(arm.transitionTo(targetState.getArmPosition()))
                            .andThen(grabber.transitionTo(targetState.getGrabberState()));
        }

        coralManipulatorCommand.schedule();

        return StatusCode.OK;
    }

    @Override
    protected boolean isTransitionFinished() {
        return !arm.isTransitioning()
                && !elevator.isTransitioning()
                && !grabber.isTransitioning()
                && !wrist.isTransitioning();
    }
}
