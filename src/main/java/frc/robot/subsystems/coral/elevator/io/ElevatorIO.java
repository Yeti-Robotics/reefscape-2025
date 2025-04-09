package frc.robot.subsystems.coral.elevator.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.util.akit.io.PositionalSetpointMotorIO;

public interface ElevatorIO
        extends PositionalSetpointMotorIO<ElevatorPosition, Angle> {
    boolean bottomSwitchTriggered();
    void setCurrentPositionToZero();
    void stopOutput();
}
