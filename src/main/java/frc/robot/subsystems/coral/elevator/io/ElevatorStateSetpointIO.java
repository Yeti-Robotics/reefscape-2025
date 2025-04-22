package frc.robot.subsystems.coral.elevator.io;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.util.akit.io.MeasuredMotorStateSetpointIO;
import frc.robot.util.akit.io.MotorStateSetpointIO;

public interface ElevatorStateSetpointIO extends MeasuredMotorStateSetpointIO<ElevatorPosition, Angle, AngleUnit> {
    boolean bottomSwitchTriggered();

    void setCurrentPositionToZero();

    void stopOutput();
}
