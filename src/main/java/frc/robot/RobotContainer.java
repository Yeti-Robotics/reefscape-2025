// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.coral.wrist.WristPositions;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.led.LEDPatterns;
import frc.robot.subsystems.led.LEDSubsystem;

import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.apriltag.impl.photon.PhotonAprilTagSystem;
import frc.robot.util.CommandGigaStation;

import frc.robot.util.sim.Mechanisms;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final CommandXboxController primaryXboxController;
    public final CommandXboxController secondaryXboxController;
    private final CommandJoystick simJoy = new CommandJoystick(2);
    public final CommandGigaStation gigaStation;

    //    @Logged(name = "Vision/Limelight")
    //    public final LimelightAprilTagSystem limelight;

    @Logged(name = "Drivetrain")
    public CommandSwerveDrivetrain drivetrain;

    public LEDSubsystem leds;

    @Logged(name = "CoralManipulator")
    final CoralManipulatorSystem coralManipulator;

    @Logged(name = "Climber")
    public ClimberSubsystem climber;

    private final SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final Mechanisms mechanisms;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        primaryXboxController = new CommandXboxController(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
        secondaryXboxController =
                new CommandXboxController(Constants.SECONDARY_XBOX_CONTROLLER_PORT);
        gigaStation = new CommandGigaStation(Constants.GIGA_PORT);
        drivetrain = TunerConstants.createDrivetrain();

        //        limelight = new LimelightAprilTagSystem("limelight", drivetrain);
        coralManipulator = new CoralManipulatorSystem();
        climber = new ClimberSubsystem();
        leds = new LEDSubsystem();
        new Trigger(coralManipulator::isTransitioning)
                .whileFalse(leds.selectAnimationCommand(coralManipulator::getCurrentState));
        mechanisms = new Mechanisms();

        configureBindings();
        configureLEDTriggers();

        var autoCommands = new AutoCommands(coralManipulator, drivetrain);

        autoChooser = autoCommands.buildAutoCommandChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Set standard deviations to prevent jitter
        // odo data is more trustworthy, lower stddev
        drivetrain.setStateStdDevs(VecBuilder.fill(0.03, 0.03, 1));
        // vision data can vary, so higher stddev
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
    final StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault().getStructTopic("/Pose", Pose2d.struct).publish();

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
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

        primaryXboxController
                .x()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        primaryXboxController
                .leftBumper()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));
        primaryXboxController
                .rightBumper()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        primaryXboxController.leftTrigger().onTrue(coralManipulator.selectQueuedStateCommand());
        primaryXboxController.rightTrigger().onTrue((coralManipulator.scoreState()));
        gigaStation
                .topMiddleSwitch()
                .onTrue(
                        coralManipulator
                                .transitionTo(CoralManipulatorState.CLIMB)
                                .alongWith(leds.runPattern(LEDPatterns.FADING_BLUE_SCROLL)));
        gigaStation
                .bottomRightGreen()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        gigaStation.bottomRightWhite().onTrue(coralManipulator.grabber.transitionTo(GrabberState.OFF));
        gigaStation
                .bottomLeftGreen()
                .onTrue(coralManipulator.grabber.transitionTo(GrabberState.ALGAE_SHOOT));
        gigaStation.topLeftBlue().onTrue(coralManipulator.setQueueState(CoralManipulatorState.L1));
        gigaStation.topRightBlue().onTrue(coralManipulator.setQueueState(CoralManipulatorState.L2));
        gigaStation.topLeftWhite().onTrue(coralManipulator.setQueueState(CoralManipulatorState.L3));
        gigaStation.topRightWhite().onTrue(coralManipulator.setQueueState(CoralManipulatorState.L4));
        gigaStation
                .topRightSwitch()
                .onTrue(leds.runPattern(LEDPatterns.NICK_MODE))
                .onFalse(leds.runPattern(LEDPatterns.YETI_BLUE_PATTERN));
        gigaStation
                .bottomLeftWhite()
                .whileTrue(coralManipulator.grabber.transitionTo(GrabberState.ROLL_IN));
        gigaStation
                .bottomRightSwitch()
                .onTrue(coralManipulator.wrist.transitionTo(WristPositions.SAFE))
                .onFalse(coralManipulator.wrist.transitionTo(WristPositions.FLIP_SAFE));

        gigaStation
                .bottomMiddleSwitch()

                .onTrue(coralManipulator.setMode(CoralManipulatorSystem.Mode.ALGAE))
                .onFalse(coralManipulator.setMode(CoralManipulatorSystem.Mode.CORAL));
        gigaStation.bottomLeftBlue().whileTrue(climber.spinClimber(climber.climbSpeed));
        gigaStation.bottomRightBlue().whileTrue(climber.spinClimber(climber.unClimbSpeed));
        gigaStation
                .topLeftGreen()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.ALGAE_HIGH));
        gigaStation
                .topRightGreen()
                .whileTrue(coralManipulator.grabber.transitionTo(GrabberState.ALL_IN));

        coralManipulator.grabber.hasCoralTrigger.onTrue(
                coralManipulator
                        .transitionTo(CoralManipulatorState.STOWED)
                        .unless(DriverStation::isAutonomous)
                        .unless(coralManipulator::isAlgaeMode)
                        .unless(
                                () ->
                                        coralManipulator.getCurrentState()
                                                        == CoralManipulatorState.SCORE_L3
                                                || coralManipulator.getCurrentState()
                                                        == CoralManipulatorState.SCORE_L2
                                                || coralManipulator.getCurrentState()
                                                        == CoralManipulatorState.L3
                                                || coralManipulator.getCurrentState()
                                                        == CoralManipulatorState.L2));

        simJoy.button(1).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L1));
        simJoy.button(2).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L2));
        simJoy.button(3).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L3));
        simJoy.button(4).onTrue(coralManipulator.transitionTo(CoralManipulatorState.L4));
        simJoy.button(5).onTrue(coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        simJoy.button(6).onTrue(coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));
        simJoy.button(7).onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        simJoy.button(8).onTrue(coralManipulator.transitionTo(CoralManipulatorState.CLIMB));
        simJoy.button(9).onTrue(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4));
        simJoy.button(10)
                .onTrue(leds.runPattern(LEDPatterns.NICK_MODE))
                .onFalse(leds.runPattern(LEDPatterns.YETI_BLUE_PATTERN));
        simJoy.button(11).onTrue(coralManipulator.transitionTo(CoralManipulatorState.ALGAE_HIGH));
        simJoy.button(12).onTrue(runOnce(() -> leds.addProgress()));
        simJoy.button(13).onTrue(runOnce(() -> leds.subtractProgress()));
        simJoy.button(15).onTrue(leds.runPattern(LEDPatterns.FADING_BLUE_SCROLL));
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(
                coralManipulator.elevator.getCurrentPosition(),
                coralManipulator.arm.getCurrentPosition(),
                coralManipulator.wrist.getCurrentPosition(),
                true);
        mechanisms.publishComponentPoses(
                coralManipulator.elevator.getTargetPosition(),
                coralManipulator.arm.getTargetPosition(),
                coralManipulator.wrist.getTargetPosition(),
                false);
        mechanisms.updateElevatorArmMech(
                coralManipulator.elevator.getCurrentPosition(),
                coralManipulator.arm.getCurrentPosition());
    }

    private void configureLEDTriggers() {
        Trigger coralTrigger =
                new Trigger(coralManipulator.grabber::hasCoral)
                        .and(DriverStation::isDisabled)
                        .onTrue(runOnce(() -> leds.addProgress()).ignoringDisable(true))
                        .onFalse(runOnce(() -> leds.subtractProgress()).ignoringDisable(true));
        Trigger elevatorTrigger =
                new Trigger(coralManipulator.elevator::getMagSwitch)
                        .and(DriverStation::isDisabled)
                        .onTrue(runOnce(() -> leds.addProgress()).ignoringDisable(true))
                        .onFalse(runOnce(() -> leds.subtractProgress()).ignoringDisable(true));
        Trigger drivetrainTrigger =
                drivetrain
                        .zeroedWheels
                        .and(DriverStation::isDisabled)
                        .onTrue(runOnce(() -> leds.addProgress()).ignoringDisable(true))
                        .onFalse(runOnce(() -> leds.subtractProgress()).ignoringDisable(true));
        Trigger batteryTrigger =
                new Trigger(() -> RobotController.getBatteryVoltage() > 12.5)
                        .and(DriverStation::isDisabled)
                        .onTrue(runOnce(() -> leds.addProgress()).ignoringDisable(true))
                        .onFalse(runOnce(() -> leds.subtractProgress()).ignoringDisable(true));
        if (coralTrigger.getAsBoolean()) {
            leds.addProgress();
        }
        if (elevatorTrigger.getAsBoolean()) {
            leds.addProgress();
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
