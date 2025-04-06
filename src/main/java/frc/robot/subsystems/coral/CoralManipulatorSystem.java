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
import frc.robot.subsystems.coral.wrist.WristPositions;
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

    public enum Side {
        SCORE,
        CLIMB
    }

    CoralManipulatorSystem.Side side = Side.SCORE;

    public CoralManipulatorSystem() {
        super(CoralManipulatorState.IDLE);
    }

    private CoralManipulatorState queuedState = CoralManipulatorState.IDLE;

    @Logged(name = "States/Queued State")
    public String getCQueuedState() {
        return queuedState.toString();
    }

    public CoralManipulatorState getQueuedState() {
        if (side == Side.CLIMB) {
            return switch (queuedState) {
                case L3 -> CoralManipulatorState.CLIMB_L3;
                case L4 -> CoralManipulatorState.CLIMB_L4;
                default -> queuedState;
            };
        }

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

    public boolean isElevMovingUp(CoralManipulatorState targetState) {
        return targetState
                .getElevatorPosition()
                .getHeight()
                .gt(getCurrentState().getElevatorPosition().getHeight());
    }

    public boolean isMovingL2(CoralManipulatorState targetState) {
        return targetState == CoralManipulatorState.L2 && arm.getCurrentState() != ArmPosition.DOWN;
    }

    public boolean isWristFirst(CoralManipulatorState targetState) {
        return getCurrentState().getWristPosition() == WristPositions.UNSAFE
                || targetState.getWristPosition() == WristPositions.SAFE;
    }

    public boolean isArmInDanger(CoralManipulatorState targetState) {
        return arm.currentStateSignal().getValue().lt(ArmPosition.AWAY.getAngle())
                || targetState.getArmPosition().getAngle().lt(ArmPosition.AWAY.getAngle());
    }

    public boolean isIntaking(CoralManipulatorState targetState) {
        return (targetState == CoralManipulatorState.HP_INTAKE
                        || targetState == CoralManipulatorState.GROUND_INTAKE
                                && getCurrentState() == CoralManipulatorState.STOWED)
                || (targetState == CoralManipulatorState.STOWED
                                && getCurrentState() == CoralManipulatorState.HP_INTAKE
                        || getCurrentState() == CoralManipulatorState.GROUND_INTAKE);
    }

    public void queueState(CoralManipulatorState state) {
        queuedState = state;
    }

    public Command selectQueuedStateCommand() {
        return new SelectCommand(
                Map.of(
                        CoralManipulatorState.L1,
                        transitionTo(CoralManipulatorState.L1),
                        CoralManipulatorState.L2,
                        transitionTo(CoralManipulatorState.L2),
                        CoralManipulatorState.L3,
                        transitionTo(CoralManipulatorState.L3),
                        CoralManipulatorState.CLIMB_L3,
                        transitionTo(CoralManipulatorState.CLIMB_L3),
                        CoralManipulatorState.L4,
                        transitionTo(CoralManipulatorState.L4),
                        CoralManipulatorState.CLIMB_L4,
                        transitionTo(CoralManipulatorState.CLIMB_L4)),
                this::getQueuedState);
    }

    public Command scoreState() {
        return new SelectCommand(
                Map.of(
                        CoralManipulatorState.L1, transitionTo(CoralManipulatorState.SCORE_L1),
                        CoralManipulatorState.L2, transitionTo(CoralManipulatorState.SCORE_L2),
                        CoralManipulatorState.L3, transitionTo(CoralManipulatorState.SCORE_L3),
                        CoralManipulatorState.CLIMB_L3,
                                transitionTo(CoralManipulatorState.SCORE_CLIMB_L3),
                        CoralManipulatorState.L4, transitionTo(CoralManipulatorState.SCORE_L4),
                        CoralManipulatorState.CLIMB_L4,
                                transitionTo(CoralManipulatorState.SCORE_CLIMB_L4)),
                this::getQueuedState);
    }

    public Command setQueueState(CoralManipulatorState queuedState) {
        return runOnce(() -> queueState(queuedState));
    }

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand;

        if (getCurrentState() == targetState) return StatusCode.OK;

        if (isIntaking(targetState) || !isElevMovingUp(targetState)) {
            if (targetState == CoralManipulatorState.STOWED) {
                if (getCurrentState() == CoralManipulatorState.SCORE_L3) {
                    coralManipulatorCommand =
                            arm.transitionTo(targetState.getArmPosition())
                                    .andThen(
                                            elevator.transitionTo(
                                                    targetState.getElevatorPosition()))
                                    .andThen(grabber.transitionTo(targetState.getGrabberState()));
                } else {
                    coralManipulatorCommand =
                            grabber.transitionTo(targetState.getGrabberState())
                                    .alongWith(arm.transitionTo(targetState.getArmPosition()))
                                    .alongWith(
                                            elevator.transitionTo(
                                                    targetState.getElevatorPosition()));
                }
            } else {
                coralManipulatorCommand =
                        grabber.transitionTo(targetState.getGrabberState())
                                .alongWith(arm.transitionTo(targetState.getArmPosition()))
                                .andThen(elevator.transitionTo(targetState.getElevatorPosition()));
            }
        } else {
            coralManipulatorCommand =
                    grabber.transitionTo(targetState.getGrabberState())
                            .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                            .andThen(arm.transitionTo(targetState.getArmPosition()));
        }

        if (isWristFirst(targetState)) {
            if (getCurrentState() == CoralManipulatorState.GROUND_INTAKE) {
                coralManipulatorCommand =
                        arm.transitionTo(ArmPosition.AWAY_BUMPER)
                                .andThen(wrist.transitionTo(targetState.getWristPosition()))
                                .andThen(coralManipulatorCommand);
            } else {
                coralManipulatorCommand =
                        wrist.transitionTo(targetState.getWristPosition())
                                .andThen(coralManipulatorCommand);
            }
        } else {
            coralManipulatorCommand =
                    coralManipulatorCommand.andThen(
                            wrist.transitionTo(targetState.getWristPosition()));
        }

        coralManipulatorCommand =
                wrist.transitionTo(getCurrentState().getWristPosition())
                        .andThen(coralManipulatorCommand);

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

    public Command setClimberSide(Side side) {
        return runOnce(() -> this.side = side);
    }

//    public Command toggleClimberSide() {
//        return runOnce(() -> this.side = this.side == Side.SCORE ? Side.CLIMB : Side.SCORE);
//    }
}
