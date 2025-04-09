package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.elevator.io.ElevatorIO;
import frc.robot.subsystems.coral.elevator.io.ElevatorInputs;
import frc.robot.util.akit.io.InputLoggingIO;
import frc.robot.util.state.StateSubsystem;

public class ElevatorSubsystem extends StateSubsystem<Angle, ElevatorPosition, ElevatorIO>
        implements InputLoggingIO<ElevatorInputs> {
    public ElevatorSubsystem(ElevatorIO io) {
        super(io);

        new Trigger(io::bottomSwitchTriggered)
                .debounce(2)
                .onTrue(
                        runOnce(io::setCurrentPositionToZero)
                                .andThen(transitionTo(ElevatorPosition.BOTTOM)));

        new Trigger(() -> io.isAtSetpoint(ElevatorPosition.BOTTOM)).onTrue(runOnce(io::stopOutput));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.targetSetpoint = getTargetState();
    }

    public Angle position() {
        return io.getPosition();
    }
}
