package frc.robot.subsystems.drivetrain.logging;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.module.SwerveModuleIOTalonFX;
import frc.robot.util.akit.LoggingUtils;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.concurrent.locks.ReentrantLock;

public class CommandSwerveDrivetrainLogging {
    public static ReentrantLock odometryReadLock = new ReentrantLock();

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
        odometryReadLock.lock();
        LoggingUtils.logInputs("Drive/Gyro", gyro, gyroInputs);

        for (int i = 0; i < swerveModules.length; i++) {
            LoggingUtils.logInputs("Drive/Module " + i, swerveModules[i], swerveModuleLogs[i]);
        }
        odometryReadLock.unlock();
    }

    @AutoLogOutput(key = "Drive/SwerveChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return commandSwerveDrivetrain.getState().Speeds;
    }

    @AutoLogOutput(key = "Drive/SwerveModuleStates")
    public SwerveModuleState[] getSwerveModuleStates() {
        return commandSwerveDrivetrain.getState().ModuleStates;
    }

    @AutoLogOutput(key = "Drive/SwerveModuleTargets")
    public SwerveModuleState[] getSwerveModuleTargetStates() {
        return commandSwerveDrivetrain.getState().ModuleTargets;
    }

    @AutoLogOutput(key = "Drive/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return commandSwerveDrivetrain.getState().Pose;
    }

    @AutoLogOutput(key = "Drive/OdometryFrequency")
    public double getOdometryUpdateFrequency() {
        return 1.0 / commandSwerveDrivetrain.getState().OdometryPeriod;
    }
}
