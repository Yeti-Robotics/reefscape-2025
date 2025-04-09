package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.arm.io.ArmIOSimulatedTalonFX;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.elevator.io.ElevatorIOSimulatedTalonFX;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.subsystems.coral.grabber.io.GrabberIOTalonFX;
import frc.robot.subsystems.coral.wrist.WristPosition;
import frc.robot.subsystems.coral.wrist.WristSubsystem;
import frc.robot.subsystems.coral.wrist.io.WristIOTalonFX;
import frc.robot.util.state.TransitionableSubsystem;
import java.util.Map;

public class CoralManipulatorSystem extends SubsystemBase
        implements TransitionableSubsystem<CoralManipulatorState> {
    public final ArmSubsystem arm = new ArmSubsystem(new ArmIOSimulatedTalonFX());

    public final ElevatorSubsystem elevator =
            new ElevatorSubsystem(new ElevatorIOSimulatedTalonFX());

    public final GrabberSubsystem grabber = new GrabberSubsystem(new GrabberIOTalonFX());

    public final WristSubsystem wrist = new WristSubsystem(new WristIOTalonFX());

    private CoralManipulatorState queuedState;

    public boolean isElevMovingUp(CoralManipulatorState targetState) {
        return targetState.getElevatorPosition().getSetpoint().gt(elevator.position());
    }

    public boolean isMovingL2(CoralManipulatorState targetState) {
        return targetState == CoralManipulatorState.L2 && !arm.isAt(ArmPosition.DOWN);
    }

    public boolean isWristFirst(CoralManipulatorState targetState) {
        return wrist.isAt(WristPosition.UNSAFE)
                || targetState.getWristPosition() == WristPosition.SAFE;
    }

    public boolean isArmInDanger(CoralManipulatorState targetState) {
        return arm.position().lt(ArmPosition.AWAY.getSetpoint())
                || targetState.getArmPosition().getSetpoint().lt(ArmPosition.AWAY.getSetpoint());
    }

    public boolean isIntaking(CoralManipulatorState targetState) {
        return (targetState == CoralManipulatorState.HP_INTAKE
                        || targetState == CoralManipulatorState.GROUND_INTAKE
                                && isAt(CoralManipulatorState.STOWED))
                || (targetState == CoralManipulatorState.STOWED
                                && isAt(CoralManipulatorState.HP_INTAKE)
                        || isAt(CoralManipulatorState.GROUND_INTAKE));
    }

    public boolean isAt(CoralManipulatorState state) {
        return arm.isAt(state.getArmPosition())
                && elevator.isAt(state.getElevatorPosition())
                && grabber.isAt(state.getGrabberState())
                && wrist.isAt(state.getWristPosition());
    }

    public void queueState(CoralManipulatorState state) {
        queuedState = state;
    }

    public CoralManipulatorState getQueuedState() {
        return queuedState;
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
    public void periodic() {
        if (Robot.isSimulation()) {
            CoralManipulatorMechanismVisualizer.getInstance()
                    .update(arm.position(), elevator.position());
        }
    }

    @Override
    public Command transitionTo(CoralManipulatorState targetState) {
        Command coralManipulatorCommand;

        if (isIntaking(targetState) || !isElevMovingUp(targetState)) {
            if (targetState == CoralManipulatorState.STOWED) {
                if (isAt(CoralManipulatorState.SCORE_L3)) {
                    coralManipulatorCommand =
                            arm.transitionTo(targetState.getArmPosition())
                                    .andThen(
                                            elevator.transitionTo(
                                                    targetState.getElevatorPosition()))
                                    .andThen(grabber.transitionTo(targetState.getGrabberState()));
                } else {
                    coralManipulatorCommand =
                            arm.transitionTo(targetState.getArmPosition())
                                    .alongWith(
                                            elevator.transitionTo(
                                                    targetState.getElevatorPosition()))
                                    .andThen(grabber.transitionTo(targetState.getGrabberState()));
                }
            } else {
                coralManipulatorCommand =
                        arm.transitionTo(targetState.getArmPosition())
                                .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                                .andThen(grabber.transitionTo(targetState.getGrabberState()));
            }
        } else {
            coralManipulatorCommand =
                    grabber.transitionTo(targetState.getGrabberState())
                            .andThen(elevator.transitionTo(targetState.getElevatorPosition()))
                            .andThen(arm.transitionTo(targetState.getArmPosition()));
        }

        if (isWristFirst(targetState)) {
            if (isAt(CoralManipulatorState.GROUND_INTAKE)) {
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

        coralManipulatorCommand = wrist.holdPosition().andThen(coralManipulatorCommand);

        coralManipulatorCommand.schedule();

        return coralManipulatorCommand;
    }
}
