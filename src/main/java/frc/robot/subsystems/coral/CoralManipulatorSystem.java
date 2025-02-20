package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;
import frc.robot.util.state.StatefulSubsystem;

public class CoralManipulatorSystem extends StatefulSubsystem<CoralManipulatorState> {
    public final ArmSubsystem arm = new ArmSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final GrabberSubsystem grabber = new GrabberSubsystem();

    public CoralManipulatorSystem() {
        super(CoralManipulatorState.IDLE);
    }

    @Override
    protected void runPeriodic() {
        SmartDashboard.putString("Coral state", getCurrentState().toString());
        SmartDashboard.putBoolean("Arm transitioning?", arm.isTransitioning());
        SmartDashboard.putBoolean("Elevator transitioning?", elevator.isTransitioning());
        SmartDashboard.putString("Arm transition state", arm.getCurrentState().toString());
        SmartDashboard.putString("Elevator transition state", elevator.getCurrentState().toString());
    }

    @Override
    protected StatusCode initializeTransition(CoralManipulatorState targetState) {
        Command coralManipulatorCommand = arm.transitionTo(targetState.getArmPosition())
                .alongWith(elevator.transitionTo(targetState.getElevatorPosition()))
                .alongWith(grabber.transitionTo(targetState.getGrabberState()));

        if (getCurrentState().getElevatorPosition().getHeight().lt(ElevatorPosition.SAFE_POSITION.getHeight())) {
            coralManipulatorCommand = elevator.transitionTo(ElevatorPosition.SAFE_POSITION)
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
