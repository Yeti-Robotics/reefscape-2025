package frc.robot.subsystems.coral.arm.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.util.akit.io.InputLoggingIO;
import frc.robot.util.akit.io.PositionalMotorIO;
import frc.robot.util.akit.io.SetpointMotorIO;

public interface ArmIO extends InputLoggingIO<ArmInputs>, SetpointMotorIO<ArmPosition, Angle> {}
