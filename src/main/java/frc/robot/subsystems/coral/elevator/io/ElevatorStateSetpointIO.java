package frc.robot.subsystems.coral.elevator.io;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.util.akit.io.StateSetpointIO;

public interface ElevatorStateSetpointIO extends StateSetpointIO<ElevatorPosition, Angle, AngleUnit> {
    boolean bottomSwitchTriggered();
}
