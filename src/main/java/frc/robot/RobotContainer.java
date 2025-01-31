// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public CoralIntake coralIntake = new CoralIntake();

    public final CommandXboxController joystick = new CommandXboxController(1);
    final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
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
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityY(
                                                joystick.getLeftY()
                                                        * TunerConstants.kSpeedAt12Volts
                                                                .magnitude())
                                        .withVelocityX(
                                                -joystick.getLeftX()
                                                        * TunerConstants.kSpeedAt12Volts
                                                                .magnitude())
                                        .withRotationalRate(
                                                -joystick.getRightX()
                                                        * TunerConstants.MaFxAngularRate)));
        joystick.b().whileTrue(coralIntake.spinClawBackward());
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
