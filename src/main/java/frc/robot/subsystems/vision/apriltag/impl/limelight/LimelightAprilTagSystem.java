package frc.robot.subsystems.vision.apriltag.impl.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private final String limelightName;
    private AprilTagResults aprilTagResults;
    private LimelightHelpers.PoseEstimate poseEstimate;

    private LimelightHelpers.LimelightTarget_Fiducial currentBestDetection;
    private double currentBestDetectionDistance = Double.POSITIVE_INFINITY;

    public LimelightAprilTagSystem(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public void periodic() {
        currentBestDetection = null;
        currentBestDetectionDistance = Double.POSITIVE_INFINITY;

        double yawPlaceholder = 0; // TODO: need to add drivetrain values
        LimelightHelpers.SetRobotOrientation(
                limelightName,
                yawPlaceholder,
                yawPlaceholder,
                yawPlaceholder,
                yawPlaceholder,
                yawPlaceholder,
                yawPlaceholder);
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        LimelightHelpers.LimelightResults results =
                LimelightHelpers.getLatestResults(limelightName);

        if (!results.valid) {
            aprilTagResults = null;
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
    public Optional<AprilTagPose> getEstimatedPose() {
        return Optional.ofNullable(poseEstimate)
                .map(e -> new AprilTagPose(e.pose, e.tagCount, e.timestampSeconds));
    }

    @Override
    public Optional<AprilTagDetection> getBestDetection() {
        return Optional.ofNullable(currentBestDetection).map(this::mapToDetection);
    }

    @Override
    public void onlyTrackTags(int... fiducialIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, fiducialIDs);
    }

    private AprilTagDetection mapToDetection(LimelightHelpers.LimelightTarget_Fiducial aprilTag) {
        return new AprilTagDetection(
                (int) aprilTag.fiducialID,
                aprilTag.getRobotPose_FieldSpace2D(),
                aprilTag.getTargetPose_RobotSpace2D(),
                0 // we can trust MegaTag2, as it eliminates pose ambiguity
                );
    }
}
