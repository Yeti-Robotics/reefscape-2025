// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.algae.AlgaeArm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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

    public final CommandXboxController xboxController;
    final CommandSwerveDrivetrain drivetrain;
    ElevatorSubsystem elevatorSubsystem;
    Tray tray;
    AlgaeArm algaeArm;
    Climber climber;
    public static LEDSubsystem leds;
    public ProgressBar progressBar;
    CANcoder wheel1;
    CANcoder wheel2;
    CANcoder wheel3;
    CANcoder wheel4;
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private Joystick simJoy = new Joystick(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        elevatorSubsystem = new ElevatorSubsystem();
        leds = new LEDSubsystem();
        progressBar = new ProgressBar(leds);
        tray = new Tray();
        algaeArm = new AlgaeArm();
        drivetrain = TunerConstants.createDrivetrain();
        climber = new Climber();
        wheel1 = drivetrain.getCANcoder(0);
        wheel2 = drivetrain.getCANcoder(1);
        wheel3 = drivetrain.getCANcoder(2);
        wheel4 = drivetrain.getCANcoder(3);
        configureBindings();
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
        xboxController.a().onTrue(runOnce(() -> progressBar.subtractProgress()));
        xboxController.b().onTrue(runOnce(() -> progressBar.addProgress()));
    }

    private void configureTriggers() {
        new Trigger(tray::isCoralInTray)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(elevatorSubsystem::getMagSwitch)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(() -> isWheelZeroed(wheel1))
                .and(() -> isWheelZeroed(wheel2))
                .and(() -> isWheelZeroed(wheel3))
                .and(() -> isWheelZeroed(wheel4))
                .and(DriverStation::isDisabled)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(climber::isClimberZero)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
        new Trigger(algaeArm::isArmZero)
                .onTrue(runOnce(() -> progressBar.addProgress()))
                .and(DriverStation::isDisabled)
                .onFalse(runOnce(() -> progressBar.subtractProgress()));
    }

    public boolean isWheelZeroed(CANcoder wheel) {
        double position = wheel.getPosition().refresh().getValueAsDouble();
        return position >= 0 || position <= Constants.ZERO_TOLERANCE;
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
