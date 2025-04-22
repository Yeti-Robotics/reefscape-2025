package frc.robot.subsystems.coral.arm.io;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.util.akit.io.MeasuredMotorStateSetpointIO;
import frc.robot.util.akit.io.MotorStateSetpointIO;

public interface ArmStateSetpointIO extends MeasuredMotorStateSetpointIO<ArmPosition, Angle, AngleUnit> {}
