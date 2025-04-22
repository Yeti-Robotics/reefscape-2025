package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.io.ElevatorStateSetpointIO;
import frc.robot.util.state.StateSubsystem;

public class ElevatorSubsystem extends StateSubsystem<Angle, AngleUnit, ElevatorPosition, ElevatorStateSetpointIO> {
    public ElevatorSubsystem(ElevatorStateSetpointIO io) {
        super(io);
    }
}
