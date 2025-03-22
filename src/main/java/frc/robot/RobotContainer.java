// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoNamedCommands;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.apriltag.impl.limelight.LimelightAprilTagSystem;
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.vision.AprilTagSimulator;
import java.util.Optional;

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

    @Logged(name = "Vision/Limelight")
    public final LimelightAprilTagSystem limelight;

    @Logged(name = "Drivetrain")
    public CommandSwerveDrivetrain drivetrain;

    Transform3d camTrans1 =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-8),
                            Units.inchesToMeters(-7),
                            Units.inchesToMeters(22.5)),
                    new Rotation3d(0, Math.toRadians(30), Math.toRadians(90)));

    Transform3d camTrans2 =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-8),
                            Units.inchesToMeters(7),
                            Units.inchesToMeters(22.5 - 7)),
                    new Rotation3d(0, Math.toRadians(15), Math.toRadians(-90)));

    //    @Logged(name = "Vision/ScoreCam")
    //    public final PhotonAprilTagSystem reefCam1;
    //
    //    @Logged(name = "Vision/ClimbCam")
    //    public final PhotonAprilTagSystem reefCam2;

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

    AprilTagSimulator aprilTagCamSim = new AprilTagSimulator();
    private final Mechanisms mechanisms;
    private final ReefAlignCommand alignToReefCmd;
    private final AprilTagSubsystem[] aprilTagSubsystems;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        primaryXboxController = new CommandXboxController(0);
        secondaryXboxController = new CommandXboxController(1);
        drivetrain = TunerConstants.createDrivetrain();

        //        reefCam1 = new PhotonAprilTagSystem("ScoreCam", camTrans1, drivetrain);
        //        reefCam2 = new PhotonAprilTagSystem("ClimbCam", camTrans2, drivetrain);

        //        AprilTagCamSim simCam1 =
        //                AprilTagCamSimBuilder.newCamera()
        //                        .withCameraName("ScoreCam")
        //                        .withTransform(camTrans1)
        //                        .build();
        //        aprilTagCamSim.addCamera(simCam1);
        //        reefCam1.setCamera(simCam1.getCam());
        //
        //        AprilTagCamSim simCam2 =
        //                AprilTagCamSimBuilder.newCamera()
        //                        .withCameraName("ClimbCam")
        //                        .withTransform(camTrans2)
        //                        .build();
        //        aprilTagCamSim.addCamera(simCam2);
        //        reefCam2.setCamera(simCam2.getCam());

        limelight = new LimelightAprilTagSystem("limelight", drivetrain);
        climber = new ClimberSubsystem();
        coralManipulator = new CoralManipulatorSystem();
        mechanisms = new Mechanisms();
        alignToReefCmd = null;
        //                new ReefAlignCommand(
        //                        drivetrain,
        //                        reefCam1,
        //                        reefCam2,
        //                        primaryXboxController::getLeftX,
        //                        primaryXboxController::getLeftY,
        //                        primaryXboxController::getRightX);

        configureBindings();

        var namedCommands = new AutoNamedCommands(coralManipulator, alignToReefCmd);
        namedCommands.registerCommands();

        autoChooser = AutoBuilder.buildAutoChooser("driveForward");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        aprilTagSubsystems = new AprilTagSubsystem[] {limelight /*, reefCam1, reefCam2*/};

        // Set standard deviations to prevent jitter
        // odo data is more trustworthy, lower stddev
        drivetrain.setStateStdDevs(VecBuilder.fill(0.03, 0.03, 1));
        // vision data can vary, so higher stddev
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Math.toRadians(50)));
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
    public void updateVision() {
        for (AprilTagSubsystem aprilTagSubsystem : aprilTagSubsystems) {
            Optional<AprilTagPose> aprilTagPoseOpt = aprilTagSubsystem.getEstimatedPose();

            if (aprilTagPoseOpt.isPresent() && !drivetrain.isMotionBlur()) {
                AprilTagPose pose = aprilTagPoseOpt.get();

                if (pose.getNumTags() > 0) {
                    drivetrain.addVisionMeasurement(
                            pose.getEstimatedRobotPose(), pose.getTimestamp());
                }
            }
        }
    }

    //    public void updateVisionSim() {
    //        aprilTagCamSim.update(drivetrain.getState().Pose);
    //    }

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

        primaryXboxController
                .leftBumper()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));
        primaryXboxController
                .rightBumper()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        primaryXboxController.leftTrigger().onTrue(coralManipulator.selectQueuedStateCommand());
        primaryXboxController.rightTrigger().onTrue((coralManipulator.scoreState()));
        secondaryXboxController
                .rightTrigger()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.CLIMB));
        secondaryXboxController
                .a()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        secondaryXboxController.x().onTrue(coralManipulator.grabber.transitionTo(GrabberState.OFF));
        secondaryXboxController
                .b()
                .onTrue(coralManipulator.grabber.transitionTo(GrabberState.ROLL_OUT));
        secondaryXboxController
                .povUp()
                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L1));
        secondaryXboxController
                .povRight()
                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L2));
        secondaryXboxController
                .povDown()
                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L3));
        secondaryXboxController
                .povLeft()
                .onTrue(coralManipulator.setQueueState(CoralManipulatorState.L4));
        secondaryXboxController
                .leftTrigger()
                .whileTrue(coralManipulator.grabber.transitionTo(GrabberState.ROLL_IN));

        //        primaryXboxController.y().whileTrue(alignToReefCmd);
        //        secondaryXboxController.start().onTrue(alignToReefCmd.toggleBranchSelection());

        secondaryXboxController.x().onTrue(coralManipulator.grabber.transitionTo(GrabberState.OFF));
        secondaryXboxController.leftBumper().whileTrue(climber.spinClimber(climber.climbSpeed));
        secondaryXboxController.rightBumper().whileTrue(climber.spinClimber(climber.unClimbSpeed));
        secondaryXboxController
                .y()
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.ALGAEHIGH));

        coralManipulator.grabber.hasCoralTrigger.onTrue(
                coralManipulator.transitionTo(CoralManipulatorState.STOWED));
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
