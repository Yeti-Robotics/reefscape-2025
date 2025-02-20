// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

import java.util.function.BiFunction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // public final CommandXboxController xboxController;
    final CommandSwerveDrivetrain drivetrain;

    @Logged(name = "coral")
    final CoralManipulatorSystem coral;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    Mechanism2d elevatorArmMech =
            new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));
    private MechanismLigament2d liftLigament;
    private MechanismLigament2d armLigament;
    private final CommandJoystick joystick = new CommandJoystick(0);
    //private final CommandJoystick joystick1 = new CommandJoystick(1);
    //   CommandXboxController xboxController = new CommandXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        drivetrain = TunerConstants.createDrivetrain();
        coral = new CoralManipulatorSystem();
        configureBindings();
        assembleMechanisms();
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
//        drivetrain.setDefaultCommand(
//                drivetrain.applyRequest(
//                        () ->
//                                drive.withVelocityX(
//                                                -xboxController.getLeftY()
//                                                        * TunerConstants.kSpeedAt12Volts
//                                                        .magnitude())
//                                        .withVelocityY(
//                                                -xboxController.getLeftX()
//                                                        * TunerConstants.kSpeedAt12Volts
//                                                        .magnitude())
//                                        .withRotationalRate(
//                                                -xboxController.getRightX()
//                                                        * TunerConstants.MaFxAngularRate)));
//        xboxController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
//        xboxController.a().onTrue(coral.transitionTo(CoralManipulatorState.L1));

        BiFunction<Integer, Command, Command> buttonCommand = (buttonNum, andThenCmd) -> {
            return new InstantCommand(() -> SmartDashboard.putNumber("Button pressed:", buttonNum))
                    .andThen(andThenCmd);
        };

        joystick.button(1).onTrue(buttonCommand.apply(1, coral.transitionTo(CoralManipulatorState.L1)));
        joystick.button(2).onTrue(buttonCommand.apply(2, coral.transitionTo(CoralManipulatorState.L2)));
        joystick.button(3).onTrue(buttonCommand.apply(3, coral.transitionTo(CoralManipulatorState.L3)));
        joystick.button(4).onTrue(buttonCommand.apply(4, coral.transitionTo(CoralManipulatorState.L4)));
        joystick.button(5).onTrue(buttonCommand.apply(5, coral.transitionTo(CoralManipulatorState.INTAKE_CORAL)));
        joystick.button(6).onTrue(buttonCommand.apply(6, coral.transitionTo(CoralManipulatorState.STOWED)));

    }

    private void assembleMechanisms() {
        liftLigament =
                elevatorArmMech
                        .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                        .append(
                                new MechanismLigament2d(
                                        "lift",
                                        Units.feetToMeters(3),
                                        90,
                                        6,
                                        new Color8Bit(Color.kRed)));
        elevatorArmMech.getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(
                        new MechanismLigament2d(
                                "bottom",
                                Units.feetToMeters(3),
                                0,
                                6,
                                new Color8Bit(Color.kGreen)));
        armLigament =
                liftLigament.append(
                        new MechanismLigament2d(
                                "arm", Units.inchesToMeters(12), 0, 6, new Color8Bit(Color.kBlue)));
    }

    public void updateMechanisms() {
        liftLigament.setLength(coral.elevator.updateMechPos());
        armLigament.setAngle(coral.arm.updateMechPos());

        SmartDashboard.putData("Mechanisms", elevatorArmMech);
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