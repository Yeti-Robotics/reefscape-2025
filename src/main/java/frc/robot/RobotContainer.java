// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.impl.limelight.LimelightAprilTagSystem;
import frc.robot.subsystems.vision.apriltag.impl.photon.PhotonAprilTagSystem;
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.vision.AprilTagCamSim;
import frc.robot.util.sim.vision.AprilTagCamSimBuilder;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final CommandXboxController primaryXboxController;
    public final CommandJoystick gigaStation;

    @Logged(name = "Vision/Limelight")
    public final LimelightAprilTagSystem limelight;

    @Logged(name = "Drivetrain")
    public CommandSwerveDrivetrain drivetrain;

    Transform3d camTrans =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-8),
                            Units.inchesToMeters(-7),
                            Units.inchesToMeters(22.5)),
                    new Rotation3d(0, Math.toRadians(35), Math.toRadians(90)));

    @Logged(name = "Vision/ScoreCam")
    public final PhotonAprilTagSystem reefCam;

    @Logged(name = "CoralManipulator")
    final CoralManipulatorSystem coralManipulator;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final Mechanisms mechanisms;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        primaryXboxController = new CommandXboxController(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
        gigaStation = new CommandJoystick(Constants.SECONDARY_XBOX_CONTROLLER_PORT);
        AprilTagCamSim simCam =
                AprilTagCamSimBuilder.newCamera()
                        .withCameraName("YetiCam1")
                        .withTransform(camTrans)
                        .build();
        //        aprilTagCamSim.addCamera(simCam);
        //        reefCam.setCamera(aprilTagCamSim.getAprilTagCamSims().get(0).getCam());

        drivetrain = TunerConstants.createDrivetrain();
        limelight = new LimelightAprilTagSystem("limelight", drivetrain);
        reefCam = new PhotonAprilTagSystem("ScoreCam", camTrans, drivetrain);
        coralManipulator = new CoralManipulatorSystem();
        mechanisms = new Mechanisms();
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
    public void updateVision() {
        Optional<AprilTagPose> aprilTagPoseOpt = limelight.getEstimatedPose();

        if (aprilTagPoseOpt.isPresent() && !drivetrain.isMotionBlur()) {
            AprilTagPose pose = aprilTagPoseOpt.get();

            if (pose.getNumTags() > 0) {
                // drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                drivetrain.addVisionMeasurement(pose.getEstimatedRobotPose(), pose.getTimestamp());
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
        gigaStation.button(7).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L1));
        gigaStation.button(8).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L2));
        gigaStation.button(9).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L3));
        gigaStation.button(10).onTrue(coralManipulator.setQueueState(CoralManipulatorState.L4));
        primaryXboxController
                .leftTrigger()
                .whileTrue(
                        new ReefAlignCommand(
                                drivetrain,
                                reefCam,
                                primaryXboxController::getLeftX,
                                primaryXboxController::getLeftY));
        primaryXboxController.rightTrigger().onTrue((coralManipulator.scoreState()));
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(
                coralManipulator.elevator.getCurrentPosition(),
                coralManipulator.arm.getCurrentPosition(),
                true);
        mechanisms.publishComponentPoses(
                coralManipulator.elevator.getTargetPosition(),
                coralManipulator.arm.getTargetPosition(),
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
        return null;
    }
}
