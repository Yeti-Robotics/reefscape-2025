package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import java.util.Optional;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private static int commandCount = 0;
    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private final CoralManipulatorSystem coralManipulatorSystem;

    private final SwerveRequest.FieldCentricFacingAngle swerveReq =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1);
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    boolean isLeftBranch = false;
    boolean isFinished = false;

    boolean isRightCam = false;

    private static final Transform2d leftBranchTransform =
            new Transform2d(0.8, -0.2, Rotation2d.kZero);
    private static final Transform2d rightBranchTransform =
            new Transform2d(0.8, 0.2, Rotation2d.kZero);

    private static final TrapezoidProfile.Constraints profiledConstraints =
            new TrapezoidProfile.Constraints(3, 1);
    ProfiledPIDController movementXPIDController =
            new ProfiledPIDController(1.75, 0, 0, profiledConstraints);
    ProfiledPIDController movementYPIDController =
            new ProfiledPIDController(1.75, 0, 0, profiledConstraints);

    private Pose2d reefTargetPose;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            CoralManipulatorSystem coralManipulatorSystem,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.coralManipulatorSystem = coralManipulatorSystem;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;

        swerveReq.HeadingController.setPID(4, 0, 0);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        movementXPIDController.setTolerance(0.07);
        movementYPIDController.setTolerance(0.07);
    }

    public static StructPublisher<Pose2d> pose2dStructPublisher(String key) {
        return NetworkTableInstance.getDefault()
                .getStructTopic("ReefAlignCmd/" + key, Pose2d.struct)
                .publish();
    }

    public boolean isRedReef(int id) {
        return id > 5 && id < 12;
    }

    public boolean isBlueReef(int id) {
        return id > 16 && id < 23;
    }

    public Optional<AprilTagDetection> getReefCamDetection() {
        isRightCam = false;
        Optional<AprilTagDetection> detection1 = reefCam1.getBestDetection();
        Optional<AprilTagDetection> detection2 = reefCam2.getBestDetection();

        if (detection1.isPresent() && detection2.isPresent()) {
            int fiducial1 = detection1.get().getFiducialID();

            if (isRedReef(fiducial1) || isBlueReef(fiducial1)) {
                return detection1;
            } else {
                isRightCam = true;
                return detection2;
            }
        }

        return detection1.or(
                () -> {
                    isRightCam = true;
                    return detection2;
                });
    }

    public Optional<Pose2d> getBranchPoseFromTagID(int id) {
        DogLog.log("ReefAlignCmd/TagID", id);
        DogLog.log("ReefAlignCmd/isRedReef", isRedReef(id));
        DogLog.log("ReefAlignCmd/isBlueReef", isBlueReef(id));
        boolean isRedAllianceReef = isRedReef(id);

        if (!isRedAllianceReef && !isBlueReef(id)) {
            isFinished = true;
            return Optional.empty();
        }

        int branchPoseIndex = id - (isRedAllianceReef ? 7 : 18);
        Pose2d[] reefTargetFaces = isRedAllianceReef ? Reef.redCenterFaces : Reef.blueCenterFaces;

        if (branchPoseIndex > 5) {
            isFinished = true;
            cancel();
            return Optional.empty();
        }

        if (branchPoseIndex == -1) {
            branchPoseIndex = Reef.blueCenterFaces.length - 1;
        }

        if (branchPoseIndex < 0) {
            isFinished = true;
            return Optional.empty();
        }

        Pose2d reefTargetPose = reefTargetFaces[branchPoseIndex];

        return Optional.of(reefTargetPose);
    }

    StructPublisher<Pose2d> reefTargetPublisher = pose2dStructPublisher("ReefTarget");

    @Override
    public void initialize() {
        isFinished = false;
        commandCount++;

        DogLog.log("ReefAlignCmd/CommandCount", commandCount);

        Optional<AprilTagDetection> detectionOpt = getReefCamDetection();

        if (detectionOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        int fiducialId = detectionOpt.get().getFiducialID();

        Optional<Pose2d> reefTargetPoseOpt = getBranchPoseFromTagID(fiducialId);

        if (reefTargetPoseOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        reefTargetPose = reefTargetPoseOpt.get();
    }

    @Override
    public void execute() {
        DogLog.log("ReefAlignCmd/TargetPoseNull", reefTargetPose == null);

        if (isFinished) {
            return;
        }

        Pose2d reefBranchPose =
                reefTargetPose
                        .transformBy(isLeftBranch ? leftBranchTransform : rightBranchTransform)
                        .transformBy(
                                new Transform2d(
                                        0,
                                        0,
                                        isRightCam ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg));
        Pose2d drivetrainPose = commandSwerveDrivetrain.getState().Pose;
        reefTargetPublisher.set(reefBranchPose);

        double veloX =
                movementXPIDController.calculate(drivetrainPose.getX(), reefBranchPose.getX());

        double veloY =
                movementYPIDController.calculate(drivetrainPose.getY(), reefBranchPose.getY());

        DogLog.log("ReefAlignCmd/XVelocity", veloX);
        DogLog.log("ReefAlignCmd/YVelocity", veloY);

        //        veloX = MathUtil.clamp(veloX, -3, 3);
        //        veloY = MathUtil.clamp(veloY, -3, 3);

        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withVelocityX(-veloX)
                        .withVelocityY(-veloY)
                        .withTargetDirection(reefBranchPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
        reefTargetPose = null;
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
