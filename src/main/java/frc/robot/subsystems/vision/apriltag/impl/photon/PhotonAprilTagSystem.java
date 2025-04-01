package frc.robot.subsystems.vision.apriltag.impl.photon;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    private AprilTagResults aprilTagResults;
    private double maxAmbiguity = 0.3;
    private PhotonTrackedTarget currentBestDetection;
    private double currentBestDetectionTimestamp;

    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    public PhotonAprilTagSystem(
            String cameraName,
            Transform3d cameraTransform,
            CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraTransform = cameraTransform;
        this.photonPoseEstimator =
                new PhotonPoseEstimator(
                        AprilTagConstants.APRIL_TAG_FIELD_LAYOUT,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cameraTransform);
        this.drivetrain = commandSwerveDrivetrain;

        photonPoseEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        photonPoseEstimator.setLastPose(drivetrain.getState().Pose);

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        List<AprilTagDetection> aprilTagDetections = new ArrayList<>();

        double earliestTimestamp = Double.POSITIVE_INFINITY;
        double highestLatency = 0;

        for (PhotonPipelineResult result : results) {
            estimatedRobotPose =
                    photonPoseEstimator.update(
                            result, camera.getCameraMatrix(), camera.getDistCoeffs());

            earliestTimestamp = Math.min(earliestTimestamp, result.getTimestampSeconds());
            highestLatency = Math.max(highestLatency, result.metadata.getLatencyMillis());

            if (result.hasTargets()) {
                PhotonTrackedTarget bestDetection = result.getBestTarget();

                if (result.getTimestampSeconds() > currentBestDetectionTimestamp) {
                    currentBestDetection = bestDetection;
                    currentBestDetectionTimestamp = result.getTimestampSeconds();
                }

                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() != -1) {
                        mapToDetection(target).ifPresent(aprilTagDetections::add);
                    }
                }
            }
        }

        aprilTagResults =
                new AprilTagResults(earliestTimestamp, highestLatency, aprilTagDetections);
    }

    public void setCamera(PhotonCamera camera) {
        this.camera = camera;
    }

    private Optional<AprilTagDetection> mapToDetection(PhotonTrackedTarget target) {
        if (target.getPoseAmbiguity() > maxAmbiguity) {
            return Optional.empty();
        }

        Optional<Pose3d> optAprilTagPose =
                AprilTagConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);

        if (optAprilTagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d aprilTagPose = optAprilTagPose.get();

        Pose3d robotPose =
                PhotonUtils.estimateFieldToRobotAprilTag(
                        target.bestCameraToTarget, aprilTagPose, cameraTransform.inverse());

        Pose3d targetPose =
                new Pose3d().transformBy(cameraTransform).transformBy(target.bestCameraToTarget);

        return Optional.of(
                new AprilTagDetection(
                        target.getFiducialId(),
                        robotPose.toPose2d(),
                        targetPose.toPose2d(),
                        target.getPoseAmbiguity()));
    }

    @Override
    public Optional<AprilTagResults> getResults() {
        return Optional.of(aprilTagResults);
    }

    @Override
    public Optional<AprilTagPose> getEstimatedPose() {
        return estimatedRobotPose.map(
                e ->
                        new AprilTagPose(
                                e.estimatedPose.toPose2d(),
                                e.targetsUsed.size(),
                                e.timestampSeconds));
    }

    @Override
    public Optional<AprilTagDetection> getBestDetection() {
        return Optional.ofNullable(currentBestDetection).flatMap(this::mapToDetection);
    }

    public PhotonAprilTagSystem withAmbiguityLessThan(double ambiguity) {
        maxAmbiguity = ambiguity;
        return this;
    }
}
