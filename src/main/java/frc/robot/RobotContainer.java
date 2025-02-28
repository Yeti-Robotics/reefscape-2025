// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.ProgressBar;
import frc.robot.subsystems.tray.Tray;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final CommandXboxController primaryXboxController;

    @Logged(name = "Drivetrain")
    final CommandSwerveDrivetrain drivetrain;

    Tray tray;
    Climber climber;
    public static LEDSubsystem leds;
    public ProgressBar progressBar;

    @Logged(name = "CoralManipulator")
    final CoralManipulatorSystem coralManipulator;

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        primaryXboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        drivetrain = TunerConstants.createDrivetrain();
        coralManipulator = new CoralManipulatorSystem();
        leds = new LEDSubsystem();
        progressBar = new ProgressBar(leds);
        climber = new Climber();
        configureBindings();
        assembleMechanisms();
        configureTriggers();
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
                                                -primaryXboxController.getLeftY()
                                                        * TunerConstants.kSpeedAt12Volts
                                                                .magnitude())
                                        .withVelocityY(
                                                -primaryXboxController.getLeftX()
                                                        * TunerConstants.kSpeedAt12Volts
                                                                .magnitude())
                                        .withRotationalRate(
                                                -primaryXboxController.getRightX()
                                                        * TunerConstants.MaFxAngularRate)));
        primaryXboxController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        primaryXboxController.y().onTrue(coralManipulator.transitionTo(CoralManipulatorState.L1));
        primaryXboxController.b().onTrue(coralManipulator.transitionTo(CoralManipulatorState.L2));
        primaryXboxController.a().onTrue(coralManipulator.transitionTo(CoralManipulatorState.L3));
        primaryXboxController.x().onTrue(coralManipulator.transitionTo(CoralManipulatorState.L4));
        primaryXboxController
                .povRight()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L2));
        primaryXboxController
                .povDown()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L3));
        primaryXboxController
                .povLeft()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4));
        primaryXboxController
                .leftBumper()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        primaryXboxController
                .rightBumper()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.INTAKE_CORAL));
    }

    private void configureTriggers() {
        new Trigger(coralManipulator.grabber::hasCoral)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(coralManipulator.elevator::getMagSwitch)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        drivetrain
                .zeroedWheels
                .and(DriverStation::isDisabled)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(climber::isClimberZero)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(DriverStation::isTeleopEnabled)
                .onTrue(runOnce(() -> leds.setAnimation(LEDSubsystem.Events.IDLETELEOP)));
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
        elevatorArmMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
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
        liftLigament.setLength(coralManipulator.elevator.updateMechPos());
        armLigament.setAngle(coralManipulator.arm.updateMechPos());

        SmartDashboard.putData("Mechanisms/CoralManipulator", elevatorArmMech);
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
