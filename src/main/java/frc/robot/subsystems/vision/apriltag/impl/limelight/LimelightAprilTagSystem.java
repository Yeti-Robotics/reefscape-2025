package frc.robot.subsystems.vision.apriltag.impl.limelight;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.LimelightHelpers;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

@Logged
public class LimelightAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final String limelightName;
    private AprilTagResults aprilTagResults;
    private LimelightHelpers.PoseEstimate poseEstimate;

    private LimelightHelpers.LimelightTarget_Fiducial currentBestDetection;
    private double currentBestDetectionDistance = Double.POSITIVE_INFINITY;

    public LimelightAprilTagSystem(
            String limelightName, CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.limelightName = limelightName;
    }

    @Override
    public void periodic() {
        currentBestDetection = null;
        currentBestDetectionDistance = Double.POSITIVE_INFINITY;

        double yaw = commandSwerveDrivetrain.getRotation3d().getAngle();
        LimelightHelpers.SetRobotOrientation(limelightName, yaw, 0, 0, 0, 0, 0);
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        LimelightHelpers.LimelightResults results =
                LimelightHelpers.getLatestResults(limelightName);

        if (!results.valid) {
            return;
        }

        List<AprilTagDetection> aprilTagDetections =
                new ArrayList<>(results.targets_Fiducials.length);

        for (LimelightHelpers.LimelightTarget_Fiducial aprilTag : results.targets_Fiducials) {
            double normDistance = aprilTag.getCameraPose_TargetSpace2D().getTranslation().getNorm();

            if (normDistance < currentBestDetectionDistance) {
                currentBestDetection = aprilTag;
                currentBestDetectionDistance = normDistance;
            }

            aprilTagDetections.add(mapToDetection(aprilTag));
        }

        aprilTagResults =
                new AprilTagResults(
                        results.timestamp_LIMELIGHT_publish,
                        results.latency_pipeline,
                        aprilTagDetections);
    }

    @Override
    public Optional<AprilTagResults> getResults() {
        return Optional.ofNullable(aprilTagResults);
    }

    @Override
    public List<AprilTagPose> getEstimatedPose() {
        return poseEstimate == null
                ? Collections.emptyList()
                : Collections.singletonList(
                        new AprilTagPose(
                                poseEstimate.pose,
                                poseEstimate.tagCount,
                                poseEstimate.timestampSeconds));
    }

    @Override
    public Optional<AprilTagDetection> getBestDetection() {
        return Optional.ofNullable(currentBestDetection).map(this::mapToDetection);
    }

    //
    //    @Logged(name = "targetPoses")
    //    public List<Pose2d> getLimelightTargetPose() {
    //        return
    // aprilTagResults.getResults().stream().map(AprilTagDetection::getTargetPose).toList();
    //    }
    //
    //    @Logged(name = "robotPoses")
    //    public List<Pose2d> getRobotPose() {
    //        return
    // aprilTagResults.getResults().stream().map(AprilTagDetection::getRobotPose).toList();
    //    }
    //
    //    @Logged(name = "estimatedRobotPose")
    //    public Pose2d getEstimateRobotPose() {
    //        return poseEstimate.pose;
    //    }

    @Logged(name = "Detection translation")
    public Translation2d bestDetectionTranslatedPosePose() {
        return getBestDetection()
                .map(p -> p.getRobotToTargetPose().getTranslation())
                .orElse(new Translation2d());
    }

    private AprilTagDetection mapToDetection(LimelightHelpers.LimelightTarget_Fiducial aprilTag) {
        Pose3d robotPose = aprilTag.getRobotPose_FieldSpace();
        Transform3d camTranform =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-9.5), 0, Units.inchesToMeters(35.125)),
                        new Rotation3d(0, 0, Math.toRadians(-180)));
        Transform3d camToTarget =
                new Transform3d(
                        LimelightHelpers.getTargetPose3d_CameraSpace("limelight").getTranslation(),
                        LimelightHelpers.getTargetPose3d_CameraSpace("limelight").getRotation());
        Pose3d targetPose = robotPose.transformBy(camTranform).transformBy(camToTarget);
        return new AprilTagDetection(
                (int) aprilTag.fiducialID,
                robotPose.toPose2d(),
                targetPose.toPose2d(),
                0 // we can trust MegaTag2, as it eliminates pose ambiguity
                );
    }
}
