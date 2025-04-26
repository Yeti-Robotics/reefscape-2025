// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.io.ClimberIOTalonFX;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.coral.arm.ArmPosition;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final CommandXboxController primaryXboxController;
    public final CommandXboxController secondaryXboxController;
    public final CommandJoystick simJoy = new CommandJoystick(2);
    final CommandSwerveDrivetrain drivetrain;

    final CoralManipulatorSystem coralManipulator;
    final ClimberSubsystem climber = new ClimberSubsystem(new ClimberIOTalonFX());
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        primaryXboxController = new CommandXboxController(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
        secondaryXboxController = new CommandXboxController(Constants.SECONDARY_XBOX_CONTROLLER_PORT);
        drivetrain = TunerConstants.createDrivetrain();
        coralManipulator = new CoralManipulatorSystem();
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
        DriverStation.silenceJoystickConnectionWarning(true);
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(
                        -primaryXboxController.getLeftY() * TunerConstants.kSpeedAt12Volts.magnitude())
                .withVelocityY(-primaryXboxController.getLeftX() * TunerConstants.kSpeedAt12Volts.magnitude())
                .withRotationalRate(-primaryXboxController.getRightX() * TunerConstants.MaFxAngularRate)));

        //   simJoy.button(1).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L1));
        simJoy.button(1).onTrue(Commands.print("up").andThen(coralManipulator.arm.transitionTo(ArmPosition.UP)));
        simJoy.button(2).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L2));
        simJoy.button(3).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L3));
        simJoy.button(4).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L4));
        simJoy.button(5).onTrue(coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        simJoy.button(6).onTrue(coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));
        simJoy.button(7).onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        simJoy.button(8).onTrue(coralManipulator.transitionTo(CoralManipulatorState.CLIMB));
        simJoy.button(9).onTrue(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L3));
        simJoy.button(10)
                .onTrue(Commands.print("Pressed").andThen(coralManipulator.grabber.transitionTo(GrabberState.ROLL_IN)));
        //        secondaryXboxController
        //                .povUp()
        //                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L1));
        //        secondaryXboxController
        //                .povRight()
        //                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L2));
        //        secondaryXboxController
        //                .povDown()
        //                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L3));
        //        secondaryXboxController
        //                .povLeft()
        //                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L4));
        //
        // primaryXboxController.leftTrigger().onTrue(coralManipulator.selectQueuedStateCommand());
        //        primaryXboxController.rightTrigger().onTrue((coralManipulator.scoreState()));
    }

    public void updateMechanisms() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return null;
    }
}
