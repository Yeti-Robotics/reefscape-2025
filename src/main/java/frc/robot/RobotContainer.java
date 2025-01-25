// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorSysIdSystem;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final CommandXboxController xboxController = new CommandXboxController(1);
    final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    ElevatorSysIdSystem el = new ElevatorSysIdSystem();

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        BooleanSupplier rangeLimit = () -> {
            return false;
        };

        xboxController
                .a()
                .and(xboxController.leftBumper())
                .whileTrue(el.sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(rangeLimit));
        xboxController
                .b()
                .and(xboxController.leftBumper())
                .whileTrue(el.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(rangeLimit));
        xboxController
                .x()
                .and(xboxController.leftBumper())
                .whileTrue(el.sysIdDynamic(SysIdRoutine.Direction.kForward).until(rangeLimit));
        xboxController
                .y()
                .and(xboxController.leftBumper())
                .whileTrue(el.sysIdDynamic(SysIdRoutine.Direction.kReverse).until(rangeLimit));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
