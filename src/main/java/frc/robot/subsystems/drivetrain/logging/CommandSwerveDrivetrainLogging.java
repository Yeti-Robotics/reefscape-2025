package frc.robot.subsystems.drivetrain.logging;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIOTalonFX;
import org.littletonrobotics.junction.Logger;

public class CommandSwerveDrivetrainLogging {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModuleIO[] swerveModules;
    private final SwerveModuleIOInputsAutoLogged[] swerveModuleLogs;

    public CommandSwerveDrivetrainLogging(CommandSwerveDrivetrain drivetrain) {
        commandSwerveDrivetrain = drivetrain;
        gyro = new GyroIOPigeon2(drivetrain.getPigeon2());

        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = drivetrain.getModules();
        swerveModules = new SwerveModuleIO[modules.length];
        swerveModuleLogs = new SwerveModuleIOInputsAutoLogged[modules.length];

        for (int i = 0; i < modules.length; i++) {
            swerveModules[i] = new SwerveModuleIOTalonFX(modules[i]);
            swerveModuleLogs[i] = new SwerveModuleIOInputsAutoLogged();
        }
    }

    public void log() {
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].updateInputs(swerveModuleLogs[i]);
            Logger.processInputs("Drive/Module" + i, swerveModuleLogs[i]);
        }

        Logger.recordOutput("Drive/SwerveChassisSpeeds", commandSwerveDrivetrain.getState().Speeds);
        Logger.recordOutput("Drive/SwerveModuleStates", commandSwerveDrivetrain.getState().ModuleStates);
        Logger.recordOutput("Drive/SwerveModuleTargets", commandSwerveDrivetrain.getState().ModuleTargets);
        Logger.recordOutput("Drive/EstimatedPose", commandSwerveDrivetrain.getState().Pose);
        Logger.recordOutput("Drive/OdometryFrequency", 1.0 / commandSwerveDrivetrain.getState().OdometryPeriod);
    }
}
