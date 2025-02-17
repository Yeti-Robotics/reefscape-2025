// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
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
    final ElevatorSubsystem elevatorSubsystem;

    @Logged(name = "Arm")
    final Arm arm;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final Joystick simJoystick = new Joystick(0);
    Mechanism2d elevatorArmMech =
            new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(50));
    private MechanismLigament2d liftLigament;
    private MechanismLigament2d armLigament;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        drivetrain = TunerConstants.createDrivetrain();
        elevatorSubsystem = new ElevatorSubsystem();
        arm = new Arm();
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
    }

    private void assembleMechanisms() {
        liftLigament =
                elevatorArmMech
                        .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                        .append(
                                new MechanismLigament2d(
                                        "lift",
                                        Units.feetToMeters(1),
                                        90,
                                        6,
                                        new Color8Bit(Color.kRed)));
        armLigament =
                liftLigament.append(
                        new MechanismLigament2d(
                                "arm", Units.inchesToMeters(12), 0, 6, new Color8Bit(Color.kBlue)));
    }

    public void updateMechanisms() {
        liftLigament.setLength(elevatorSubsystem.updateMechPos());
        armLigament.setAngle(arm.updateMechPos());

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
