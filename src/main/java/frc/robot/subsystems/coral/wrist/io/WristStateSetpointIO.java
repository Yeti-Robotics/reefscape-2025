package frc.robot.subsystems.coral.wrist.io;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.wrist.WristPosition;
import frc.robot.util.akit.io.StateSetpointIO;

public interface WristStateSetpointIO extends StateSetpointIO<WristPosition, Angle, AngleUnit> {}
