package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.akit.InputLoggingIO;

public interface SwerveModuleIO extends InputLoggingIO<SwerveModuleIOInputs> {
    SwerveModuleState getModuleState();
}
