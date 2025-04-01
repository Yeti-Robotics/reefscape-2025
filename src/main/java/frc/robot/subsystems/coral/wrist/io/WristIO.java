package frc.robot.subsystems.coral.wrist.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.wrist.WristPosition;
import frc.robot.util.akit.io.PositionalSetpointMotorIO;

public interface WristIO extends PositionalSetpointMotorIO<WristPosition, Angle> {}
