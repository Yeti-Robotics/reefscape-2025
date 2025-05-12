package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.io.ElevatorIO;
import frc.robot.util.state.AbstractStateSubsystem;

public class ElevatorSubsystem extends AbstractStateSubsystem<Angle, ElevatorPosition, ElevatorIO> {
    public ElevatorSubsystem(ElevatorIO io) {
        super(io);
    }
}
