package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.AprilTagDetectionHelpers;
import java.util.Optional;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private static int commandCount = 0; // for logging purposes
    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private final CoralManipulatorSystem coralManipulatorSystem;

    private final SwerveRequest.FieldCentricFacingAngle swerveReq =
            new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    private boolean isLeftBranch = false;
    private boolean isFinished = false;
    private boolean isRightCam = false;

    private static final Transform2d leftBranchTransform =
            new Transform2d(Units.inchesToMeters(12), Units.inchesToMeters(-6.5), Rotation2d.kZero);
    private static final Transform2d rightBranchTransform =
            new Transform2d(Units.inchesToMeters(12), Units.inchesToMeters(6.5), Rotation2d.kZero);
    private static final Transform2d rightTurnTransform =
            new Transform2d(0, 0, Rotation2d.kCW_90deg);
    private static final Transform2d leftTurnTransform =
            new Transform2d(0, 0, Rotation2d.kCCW_90deg);

    PIDController movementXPIDController = new PIDController(9, 0, 0.2);
    PIDController movementYPIDController = new PIDController(9, 0, 0.2);

    // apparently profiled PID outputs a positive velo which isn't ideal for alignment
    // DO NOT USE
    //   private static final TrapezoidProfile.Constraints profiledConstraints =
    //            new TrapezoidProfile.Constraints(3, 1);
    //    ProfiledPIDController movementXPIDController =
    //            new ProfiledPIDController(1.5, 0, 0, profiledConstraints);
    //    ProfiledPIDController movementYPIDController =
    //            new ProfiledPIDController(1.5, 0, 0, profiledConstraints);

    private static final double feedforward = 0.0;
    private Pose2d reefFaceTargetPose;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            CoralManipulatorSystem coralManipulatorSystem,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.coralManipulatorSystem = coralManipulatorSystem;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;

        swerveReq.HeadingController.setPID(7, 0, 0);
        swerveReq.HeadingController.setTolerance(0.04);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        movementXPIDController.setTolerance(0.07);
        movementYPIDController.setTolerance(0.07);
        getBranchPoseFromTagID(18);
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

    public boolean isOnReef(int id) {
        return isRedReef(id) || isBlueReef(id);
    }

    public Optional<AprilTagDetection> getReefCamDetection() {
        isRightCam = false;
        Optional<AprilTagDetection> detection1 = reefCam1.getBestDetection();
        Optional<AprilTagDetection> detection2 = reefCam2.getBestDetection();

        if (detection1.isPresent() && detection2.isPresent()) {
            AprilTagDetection fiducial1 = detection1.get();
            AprilTagDetection fiducial2 = detection2.get();

            boolean fiducial1IsOnReef = isOnReef(fiducial1.getFiducialID());
            boolean fiducial2IsOnReef = isOnReef(fiducial2.getFiducialID());

            if (fiducial1IsOnReef && fiducial2IsOnReef) {
                boolean fiducial1Closer =
                        AprilTagDetectionHelpers.getDetectionDistance(
                                        fiducial1.getRobotToTargetPose())
                                < AprilTagDetectionHelpers.getDetectionDistance(
                                        fiducial2.getRobotToTargetPose());

                if (!fiducial1Closer) {
                    isRightCam = true;
                }

                return fiducial1Closer ? detection1 : detection2;
            }

            if (fiducial2IsOnReef) {
                isRightCam = true;
            }

            return fiducial1IsOnReef ? detection1 : detection2;
        }

        return detection1.or(
                () -> {
                    isRightCam = true;
                    return detection2;
                });
    }

    public Optional<Pose2d> getBranchPoseFromTagID(int id) {
        //        DogLog.log("ReefAlignCmd/TagID", id);
        //        DogLog.log("ReefAlignCmd/isRedReef", isRedReef(id));
        //        DogLog.log("ReefAlignCmd/isBlueReef", isBlueReef(id));
        boolean isRedAllianceReef = isRedReef(id);

        if (!isRedAllianceReef && !isBlueReef(id)) {
            isFinished = true;
            return Optional.empty();
        }

        int branchPoseIndex = id - (isRedAllianceReef ? 7 : 18);
        Pose2d[] reefTargetFaces = isRedAllianceReef ? Reef.redCenterFaces : Reef.blueCenterFaces;

        if (branchPoseIndex > 5) {
            isFinished = true;
            return Optional.empty();
        }

        if (branchPoseIndex == -1) {
            branchPoseIndex = reefTargetFaces.length - 1;
        }

        if (branchPoseIndex < 0) {
            isFinished = true;
            return Optional.empty();
        }

        movementXPIDController.reset();
        movementYPIDController.reset();

        Pose2d reefTargetPose = reefTargetFaces[branchPoseIndex];

        return Optional.of(reefTargetPose);
    }

    StructPublisher<Pose2d> reefTargetPublisher = pose2dStructPublisher("ReefTarget");

    @Override
    public void initialize() {
        isFinished = false;
        commandCount++;

        //  DogLog.log("ReefAlignCmd/CommandCount", commandCount);

        Optional<AprilTagDetection> detectionOpt = getReefCamDetection();

        if (detectionOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        int fiducialId = detectionOpt.get().getFiducialID();
        System.out.println("Detection id: " + fiducialId);
        Optional<Pose2d> reefTargetPoseOpt = getBranchPoseFromTagID(fiducialId);

        if (reefTargetPoseOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        reefFaceTargetPose = reefTargetPoseOpt.get();
    }

    @Override
    public void execute() {
        //   DogLog.log("ReefAlignCmd/TargetPoseNull", reefFaceTargetPose == null);

        if (isFinished) {
            return;
        }

        Pose2d reefBranchPose =
                reefFaceTargetPose
                        .transformBy(isLeftBranch ? leftBranchTransform : rightBranchTransform)
                        .transformBy(isRightCam ? rightTurnTransform : leftTurnTransform);

        Pose2d drivetrainPose = commandSwerveDrivetrain.getState().Pose;

        reefTargetPublisher.set(reefBranchPose);

        Transform2d targetTransform = new Transform2d(drivetrainPose, reefBranchPose);
        //   DogLog.log("ReefAlignCmd/TargetTransform", targetTransform);

        double veloX =
                movementXPIDController.calculate(drivetrainPose.getX(), reefBranchPose.getX());

        double veloY =
                movementYPIDController.calculate(drivetrainPose.getY(), reefBranchPose.getY());

        double veloXFeed = feedforward * Math.signum(veloX) * 0.5;
        double veloYFeed = feedforward * Math.signum(veloY) * 0.5;
        //
        //        DogLog.log("ReefAlignCmd/XVelocity", veloX);
        //        DogLog.log("ReefAlignCmd/YVelocity", veloY);
        //        DogLog.log("ReefAlignCmd/XVelocityFeed", veloXFeed);
        //        DogLog.log("ReefAlignCmd/YVelocityFeed", veloYFeed);
        //        DogLog.log("ReefAlignCmd/XError", movementXPIDController.getError());
        //        DogLog.log("ReefAlignCmd/YError", movementYPIDController.getError());

        veloX = MathUtil.clamp(veloX, -3, 3);
        veloY = MathUtil.clamp(veloY, -3, 3);

        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withVelocityX(-(veloX))
                        .withVelocityY(-(veloY))
                        .withTargetDirection(reefBranchPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
        reefFaceTargetPose = null;
        isFinished = false;
    }

    private boolean alignmentReached() {
        return movementYPIDController.atSetpoint()
                && movementXPIDController.atSetpoint()
                && swerveReq.HeadingController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return isFinished || alignmentReached();
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
