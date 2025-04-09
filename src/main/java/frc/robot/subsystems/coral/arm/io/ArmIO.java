package frc.robot.subsystems.coral.arm.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.util.akit.io.PositionalSetpointMotorIO;

public interface ArmIO extends PositionalSetpointMotorIO<ArmPosition, Angle> {}
