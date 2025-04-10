// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoNamedCommands;
import frc.robot.commands.ReefAlignPPOTF;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.coral.grabber.GrabberState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.apriltag.impl.photon.PhotonAprilTagSystem;
import frc.robot.util.PathPlannerUtils;
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.vision.AprilTagCamSim;
import frc.robot.util.sim.vision.AprilTagCamSimBuilder;
import frc.robot.util.sim.vision.AprilTagSimulator;
import java.util.List;
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
    public final CommandJoystick gigaStation;

    //    @Logged(name = "Vision/Limelight")
    //    public final LimelightAprilTagSystem limelight;

    @Logged(name = "Drivetrain")
    public CommandSwerveDrivetrain drivetrain;

    Transform3d camTrans1 =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-9.5),
                            Units.inchesToMeters(-8),
                            Units.inchesToMeters(11)),
                    new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-90)));

    Transform3d camTrans2 =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-9.5),
                            Units.inchesToMeters(10),
                            Units.inchesToMeters(11)),
                    new Rotation3d(0, Math.toRadians(-15), Math.toRadians(90)));

    @Logged(name = "Vision/RadioCam")
    public final PhotonAprilTagSystem radioCam;

    @Logged(name = "Vision/ScoreCam")
    public final PhotonAprilTagSystem scoreCam;

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
    private final ReefAlignPPOTF reefAlignPPOTF;
    private final AprilTagSubsystem[] aprilTagSubsystems;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        primaryXboxController = new CommandXboxController(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
        secondaryXboxController =
                new CommandXboxController(Constants.SECONDARY_XBOX_CONTROLLER_PORT);
        gigaStation = new CommandJoystick(Constants.GIGA_PORT);
        drivetrain = TunerConstants.createDrivetrain();

        radioCam = new PhotonAprilTagSystem("RadioCam", camTrans1, drivetrain);
        scoreCam = new PhotonAprilTagSystem("ScoreCam", camTrans2, drivetrain);
        if (Robot.isSimulation()) {
            AprilTagCamSim simCam1 =
                    AprilTagCamSimBuilder.newCamera()
                            .withCameraName("ScoreCam")
                            .withTransform(camTrans1)
                            .build();
            aprilTagCamSim.addCamera(simCam1);
            radioCam.setCamera(simCam1.getCam());

            AprilTagCamSim simCam2 =
                    AprilTagCamSimBuilder.newCamera()
                            .withCameraName("ClimbCam")
                            .withTransform(camTrans2)
                            .build();
            aprilTagCamSim.addCamera(simCam2);
            scoreCam.setCamera(simCam2.getCam());
        }

        //        limelight = new LimelightAprilTagSystem("limelight", drivetrain);
        climber = new ClimberSubsystem();
        coralManipulator = new CoralManipulatorSystem();
        mechanisms = new Mechanisms();
        reefAlignPPOTF = new ReefAlignPPOTF(drivetrain, radioCam, scoreCam);

        configureBindings();

        var namedCommands = new AutoNamedCommands(coralManipulator, reefAlignPPOTF);
        namedCommands.registerCommands();

        autoChooser = AutoBuilder.buildAutoChooser("driveForward");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        aprilTagSubsystems = new AprilTagSubsystem[] {radioCam, scoreCam};

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
    public void updateVision() {
        for (AprilTagSubsystem aprilTagSubsystem : aprilTagSubsystems) {
            List<AprilTagPose> aprilTagPoseOpt = aprilTagSubsystem.getEstimatedPose();

            if (!aprilTagPoseOpt.isEmpty() && !drivetrain.isMotionBlur()) {
                for (AprilTagPose pose : aprilTagPoseOpt) {
                    if (pose.numTags() > 0) {
                        drivetrain.addVisionMeasurement(
                                pose.estimatedRobotPose(),
                                pose.timestamp(),
                                pose.standardDeviations());
                    }
                }
            }
        }
    }

    public void updateVisionSim() {
        aprilTagCamSim.update(drivetrain.getState().Pose);
    }

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

        primaryXboxController.leftBumper().onTrue(new InstantCommand(SignalLogger::start));
        primaryXboxController.rightBumper().onTrue(new InstantCommand(SignalLogger::stop));
        primaryXboxController.leftTrigger().onTrue(coralManipulator.selectQueuedStateCommand());
        primaryXboxController.rightTrigger().onTrue((coralManipulator.scoreState()));
        primaryXboxController
                .y()
                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        //        primaryXboxController
        //                .a()
        //                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        primaryXboxController
                .b()
                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        primaryXboxController
                .x()
                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        primaryXboxController
                .a()
                .onTrue(driveForward().withTimeout(1).andThen(SwerveRequest.Idle::new));
        //        primaryXboxController.button(1).whileTrue(reefAlignPPOTF.reefAlign());
        gigaStation.button(5).onTrue(coralManipulator.transitionTo(CoralManipulatorState.CLIMB));
        gigaStation.button(18).onTrue(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
        gigaStation.button(16).onTrue(coralManipulator.grabber.transitionTo(GrabberState.OFF));
        gigaStation.button(17).onTrue(coralManipulator.grabber.transitionTo(GrabberState.ROLL_OUT));
        gigaStation.button(7).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L1));
        gigaStation.button(8).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L2));
        gigaStation.button(9).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L3));
        gigaStation.button(10).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L4));
        gigaStation
                .button(15)
                .whileTrue(coralManipulator.grabber.transitionTo(GrabberState.ROLL_IN));

        gigaStation
                .button(4)
                .onTrue(reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.RIGHT))
                .onFalse(reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.LEFT));
        gigaStation.button(13).whileTrue(climber.spinClimber(climber.climbSpeed));
        gigaStation.button(14).whileTrue(climber.spinClimber(climber.unClimbSpeed));
        gigaStation
                .button(11)
                .onTrue(coralManipulator.transitionTo(CoralManipulatorState.ALGAEHIGH));

        coralManipulator.grabber.hasCoralTrigger.onTrue(
                coralManipulator.transitionTo(CoralManipulatorState.STOWED));

        coralManipulator.grabber.doesNotHaveCoralTrigger.onTrue(
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

    public Command right1Pc() {
        Optional<PathPlannerPath> lineF = PathPlannerUtils.loadPathByName("lineF");

        return lineF.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(lineF.get())
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.L4)
                                        .withTimeout(0.5))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.STOWED));
    }

    public Command left2PcLolli() {
        Optional<PathPlannerPath> lineJ = PathPlannerUtils.loadPathByName("lineJ");
        Optional<PathPlannerPath> jToLollipop = PathPlannerUtils.loadPathByName("jToLoli");
        Optional<PathPlannerPath> lollipopToL = PathPlannerUtils.loadPathByName("loliToL");

        PathPlannerAuto auto;
        PathPlannerAuto lollipathJ =
                new PathPlannerAuto((AutoBuilder.followPath(jToLollipop.get())));
        PathPlannerAuto lollipathL =
                new PathPlannerAuto((AutoBuilder.followPath(lollipopToL.get())));

        var cmd =
                lineJ.isEmpty() || jToLollipop.isEmpty()
                        ? Commands.none()
                        : Commands.sequence(
                                AutoBuilder.followPath(lineJ.get()),
                                reefAlignPPOTF.reefAlign(),
                                coralManipulator.transitionTo(CoralManipulatorState.L4),
                                coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5),
                                AutoBuilder.followPath(jToLollipop.get())
                                        .until(coralManipulator.grabber::hasCoral),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(1),
                                AutoBuilder.followPath(lollipopToL.get()),
                                reefAlignPPOTF.reefAlign(),
                                coralManipulator.transitionTo(CoralManipulatorState.CLIMB_L4),
                                coralManipulator.transitionTo(CoralManipulatorState.SCORE_CLIMB_L4),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command right2PcLolli() {
        Optional<PathPlannerPath> lineF = PathPlannerUtils.loadPathByName("lineF");
        Optional<PathPlannerPath> fToLollipop = PathPlannerUtils.loadPathByName("fToLoli");
        Optional<PathPlannerPath> lollipopToD = PathPlannerUtils.loadPathByName("loliToD");

        PathPlannerAuto auto;

        var cmd =
                lineF.isEmpty() || fToLollipop.isEmpty() || lollipopToD.isEmpty()
                        ? Commands.none()
                        : Commands.sequence(
                                AutoBuilder.followPath(lineF.get()),
                                reefAlignPPOTF.reefAlign(),
                                coralManipulator.transitionTo(CoralManipulatorState.L4),
                                coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5)
                                        .andThen(
                                                Commands.sequence(
                                                                AutoBuilder.followPath(
                                                                        fToLollipop.get()),
                                                                coralManipulator
                                                                        .transitionTo(
                                                                                CoralManipulatorState
                                                                                        .STOWED)
                                                                        .withTimeout(0.5),
                                                                AutoBuilder.followPath(
                                                                        lollipopToD.get()),
                                                                reefAlignPPOTF.reefAlign(),
                                                                coralManipulator.transitionTo(
                                                                        CoralManipulatorState
                                                                                .CLIMB_L4),
                                                                coralManipulator.transitionTo(
                                                                        CoralManipulatorState
                                                                                .SCORE_CLIMB_L4),
                                                                coralManipulator
                                                                        .transitionTo(
                                                                                CoralManipulatorState
                                                                                        .STOWED)
                                                                        .withTimeout(0.5))
                                                        .onlyIf(
                                                                coralManipulator.grabber
                                                                        ::doesNotHaveCoral)));
        auto = new PathPlannerAuto(cmd);
        return auto;
    }

    public Command left1Pc() {
        Optional<PathPlannerPath> lineJ = PathPlannerUtils.loadPathByName("lineJ");
        return lineJ.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(lineJ.get())
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5));
    }

    public Command driveForward() {
        return drivetrain.applyRequest(
                () -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(1, 0, 0)));
    }

    public Command mid1Pc() {
        Optional<PathPlannerPath> lineG = PathPlannerUtils.loadPathByName("lineG");
        return lineG.isEmpty()
                ? Commands.none()
                : AutoBuilder.followPath(lineG.get())
                        .andThen(reefAlignPPOTF.setBranch(ReefAlignPPOTF.Branch.RIGHT))
                        .andThen(reefAlignPPOTF.reefAlign())
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.L4))
                        .andThen(coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4))
                        .andThen(
                                coralManipulator
                                        .transitionTo(CoralManipulatorState.STOWED)
                                        .withTimeout(0.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command selectedAuto = right2PcLolli();
        //        if (gigaStation.getHID().getRawButton(19)) {
        //            selectedAuto = driveForward();
        //            SmartDashboard.putString("Selected auto", "driveForward");
        //        } else if (gigaStation.getHID().getRawButton(20)) {
        //            selectedAuto = right2PcLolli();
        //            SmartDashboard.putString("Selected auto", "right2PcLolli");
        //        } else if (gigaStation.getHID().getRawButton(21)) {
        //            selectedAuto = left1Pc();
        //            SmartDashboard.putString("Selected auto", "left1Pc");
        //        }
        if (selectedAuto == null) {
            return autoChooser.getSelected();
        } else {
            return selectedAuto;
        }
    }
}
