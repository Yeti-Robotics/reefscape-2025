package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.elevator.io.ElevatorIO;
import frc.robot.util.state.StateSubsystem;

public class ElevatorSubsystem extends StateSubsystem<Angle, ElevatorPosition, ElevatorIO> {
    public ElevatorSubsystem(ElevatorIO io) {
        super(io);

        new Trigger(io::bottomSwitchTriggered)
                .debounce(2)
                .onTrue(
                        runOnce(io::setCurrentPositionToZero)
                                .andThen(transitionTo(ElevatorPosition.BOTTOM)));

        new Trigger(() -> io.isAtSetpoint(ElevatorPosition.BOTTOM)).onTrue(runOnce(io::stopOutput));
    }

    public Angle position() {
        return io.getPosition();
    }
}
