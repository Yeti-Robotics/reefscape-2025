// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositions;
import frc.robot.subsystems.coral.CoralIntake;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final CommandXboxController xboxController;
    final CommandSwerveDrivetrain drivetrain;

    @Logged(name = "Elevator")
    ElevatorSubsystem elevatorSubsystem;

    CoralIntake coralIntake;
    Arm arm;
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        drivetrain = TunerConstants.createDrivetrain();
        elevatorSubsystem = new ElevatorSubsystem();
        coralIntake = new CoralIntake();
        arm = new Arm();
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -xboxController.getLeftY()
                                                        * TunerConstants.kSpeedAt12Volts
                                                                .magnitude())
                                        .withVelocityY(
                                                -xboxController.getLeftX()
                                                        * TunerConstants.kSpeedAt12Volts
                                                                .magnitude())
                                        .withRotationalRate(
                                                -xboxController.getRightX()
                                                        * TunerConstants.MaFxAngularRate)));

        xboxController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        xboxController.povDown().onTrue(elevatorSubsystem.moveTo(ElevatorPosition.BOTTOM));
        xboxController.povRight().onTrue(elevatorSubsystem.moveTo(ElevatorPosition.TEST));
        xboxController.povUp().onTrue(elevatorSubsystem.moveTo(ElevatorPosition.LEVEL3));
        xboxController.povLeft().onTrue(elevatorSubsystem.moveTo(ElevatorPosition.LEVEL4));

        xboxController
                .a()
                .onTrue(
                        elevatorSubsystem
                                .moveTo(ElevatorPosition.BOTTOM)
                                .andThen(arm.moveTo(ArmPositions.UP)));
        xboxController
                .b()
                .onTrue(
                        elevatorSubsystem
                                .moveTo(ElevatorPosition.INTAKE)
                                .andThen(arm.moveTo(ArmPositions.INTAKE)));
        // xboxController.y().onTrue(arm.moveTo(ArmPositions.L3));
        xboxController
                .x()
                .onTrue(
                        elevatorSubsystem
                                .moveTo(ElevatorPosition.LEVEL4)
                                .andThen(arm.moveTo(ArmPositions.L4)));

        xboxController.leftBumper().whileTrue(coralIntake.spinClaw(0.5));
        xboxController.rightBumper().whileTrue(coralIntake.spinClaw(-0.2));
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
