package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.AprilTagDetectionHelpers;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public class ReefAlignPPOTF {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;

    private static final SwerveRequest.FieldCentricFacingAngle swerveReq =
            new SwerveRequest.FieldCentricFacingAngle();
    private static final SwerveRequest.ApplyRobotSpeeds robotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    private boolean isRightCam = false;

    private static final Transform2d leftBranchTransform =
            new Transform2d(Units.inchesToMeters(18), Units.inchesToMeters(-8), Rotation2d.kZero);
    private static final Transform2d rightBranchTransform =
            new Transform2d(Units.inchesToMeters(18), Units.inchesToMeters(8), Rotation2d.kZero);
    private static final Transform2d rightTurnTransform =
            new Transform2d(0, 0, Rotation2d.kCCW_90deg);
    private static final Transform2d leftTurnTransform =
            new Transform2d(0, 0, Rotation2d.kCW_90deg);

    public enum Branch {
        LEFT,
        RIGHT
    }

    Branch branch = Branch.LEFT;

    private Pose2d reefFaceTargetPose;

    public ReefAlignPPOTF(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;

        swerveReq.HeadingController.setPID(7, 0, 0);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        getBranchPoseFromTagID(18);
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

    public Command setBranch(Branch branch) {
        return Commands.runOnce(() -> this.branch = branch);
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
        DogLog.log("ReefAlignCmd/TagID", id);
        DogLog.log("ReefAlignCmd/isRedReef", isRedReef(id));
        DogLog.log("ReefAlignCmd/isBlueReef", isBlueReef(id));
        boolean isRedAllianceReef = isRedReef(id);

        if (!isRedAllianceReef && !isBlueReef(id)) {
            return Optional.empty();
        }

        int branchPoseIndex = id - (isRedAllianceReef ? 7 : 18);
        Pose2d[] reefTargetFaces = isRedAllianceReef ? Reef.redCenterFaces : Reef.blueCenterFaces;

        if (branchPoseIndex > 5) {
            return Optional.empty();
        }

        if (branchPoseIndex == -1) {
            branchPoseIndex = reefTargetFaces.length - 1;
        }

        if (branchPoseIndex < 0) {
            return Optional.empty();
        }

        Pose2d reefTargetPose = reefTargetFaces[branchPoseIndex];

        return Optional.of(reefTargetPose);
    }

    private LinearVelocity getChassisVelocity(ChassisSpeeds chassisSpeeds) {
        return MetersPerSecond.of(
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                        .getNorm());
    }

    private Command autoAlign() {
        Optional<AprilTagDetection> detectionOpt = getReefCamDetection();

        if (detectionOpt.isEmpty()) {
            return Commands.runOnce(
                    () -> commandSwerveDrivetrain.setControl(stopReq), commandSwerveDrivetrain);
        }

        int fiducialId = detectionOpt.get().getFiducialID();
        System.out.println("Detection id: " + fiducialId);
        Optional<Pose2d> reefTargetPoseOpt = getBranchPoseFromTagID(fiducialId);

        if (reefTargetPoseOpt.isEmpty()) {
            return Commands.runOnce(
                    () -> commandSwerveDrivetrain.setControl(stopReq), commandSwerveDrivetrain);
        }

        reefFaceTargetPose = reefTargetPoseOpt.get();
        Pose2d reefBranchPose =
                reefFaceTargetPose
                        .transformBy(
                                branch == Branch.LEFT ? leftBranchTransform : rightBranchTransform)
                        .transformBy(isRightCam ? rightTurnTransform : leftTurnTransform);

        DogLog.log("ReefAlignCmd/ReefTarget", reefBranchPose);

        SwerveDrivetrain.SwerveDriveState state = commandSwerveDrivetrain.getState();
        Pose2d drivetrainPose = state.Pose;

        /*
        old midpt pose
        new Pose2d(
                                reefBranchPose.getTranslation().getX() + 0.2,
                                reefBranchPose.getTranslation().getY(),
                                reefBranchPose.getRotation())
         */

        List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(drivetrainPose, reefBranchPose);

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        new PathConstraints(
                                MetersPerSecond.of(1),
                                MetersPerSecondPerSecond.of(2),
                                RadiansPerSecond.of(2 * Math.PI),
                                RadiansPerSecondPerSecond.of(4 * Math.PI)),
                        new IdealStartingState(
                                getChassisVelocity(state.Speeds),
                                commandSwerveDrivetrain.getRotation3d().toRotation2d()),
                        new GoalEndState(0.0, reefBranchPose.getRotation()));
        path.preventFlipping = true;

        PathPlannerTrajectoryState endState = new PathPlannerTrajectoryState();
        endState.pose = reefBranchPose;

        return AutoBuilder.followPath(path)
                .andThen(
                        Commands.run(
                                () ->
                                        commandSwerveDrivetrain.setControl(
                                                robotSpeeds.withSpeeds(
                                                        commandSwerveDrivetrain.driveController
                                                                .calculateRobotRelativeSpeeds(
                                                                        commandSwerveDrivetrain
                                                                                .getState()
                                                                                .Pose,
                                                                        endState))),
                                commandSwerveDrivetrain))
                .until(
                        () ->
                                new Transform2d(
                                                        commandSwerveDrivetrain.getState().Pose,
                                                        reefBranchPose)
                                                .getTranslation()
                                                .getNorm()
                                        < 0.01)
                .andThen(
                        Commands.runOnce(
                                () -> commandSwerveDrivetrain.setControl(stopReq),
                                commandSwerveDrivetrain));
    }

    public Command reefAlign() {
        return Commands.defer(this::autoAlign, Set.of(commandSwerveDrivetrain));
    }
}
