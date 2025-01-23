package frc.robot.subsystems.vision.apriltag.impl.photon;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.apriltag.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class PhotonAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private static final double DEFAULT_ACCEPTABLE_AMBIGUITY = 0.2;
    private final PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator photonPoseEstimator;

    private AprilTagResults aprilTagResults = null;
    private double maxAmbiguity = 1;
    private int[] fiducialIds = null;

    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<EstimatedRobotPose> estimatedRobotPose;

    public PhotonAprilTagSystem(String cameraName, Transform3d cameraTransform) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraTransform = cameraTransform;
        this.photonPoseEstimator = new PhotonPoseEstimator(AprilTagConstants.APRIL_TAG_FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform);

        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    private Optional<AprilTagDetection> mapToDetection(PhotonTrackedTarget target) {
        if (target.getPoseAmbiguity() > maxAmbiguity) {
            return Optional.empty();
        }

        Optional<Pose3d> optAprilTagPose = AprilTagConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);

        if (optAprilTagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d aprilTagPose = optAprilTagPose.get();

        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.bestCameraToTarget,
                aprilTagPose,
                cameraTransform
        );

        Pose3d targetPose = new Pose3d().transformBy(cameraTransform)
                .transformBy(target.bestCameraToTarget);

        return Optional.of(new AprilTagDetection(
                target.getFiducialId(),
                robotPose.toPose2d(),
                targetPose.toPose2d(),
                target.getPoseAmbiguity()
        ));
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            aprilTagResults = null;
            return;
        }

        List<AprilTagDetection> aprilTagDetections = new ArrayList<>();

        double latestTimestamp = 0;
        double highestLatency = 0;

        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result, camera.getCameraMatrix(), camera.getDistCoeffs());;

            this.estimatedRobotPose = estimatedRobotPose;

            estimatedRobotPose.ifPresent(robotPose -> photonPoseEstimator.setReferencePose(robotPose.estimatedPose));

            latestTimestamp = Math.max(latestTimestamp, result.getTimestampSeconds());
            highestLatency = Math.max(highestLatency, result.metadata.getLatencyMillis());

            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() != -1 && isFiducialOfInterest(target.getFiducialId())) {
                        mapToDetection(target).ifPresent(aprilTagDetections::add);
                    }
                }
            }
        }

        aprilTagResults = new AprilTagResults(latestTimestamp, highestLatency, aprilTagDetections);
    }

    @Override
    public Optional<AprilTagResults> getResults() {
        return Optional.of(aprilTagResults);
    }

    @Override
    public Optional<AprilTagPose> getEstimatedPose() {
        return estimatedRobotPose.map(e -> new AprilTagPose(e.estimatedPose.toPose2d(), e.targetsUsed.size(), e.timestampSeconds));
    }

    @Override
    public void filterFiducialIDs(int... ids) {
        fiducialIds = ids;
    }

    private boolean isFiducialOfInterest(int id) {
        return (fiducialIds == null || fiducialIds.length < 1) || Arrays.stream(fiducialIds).anyMatch(e -> e == id);
    }

    public PhotonAprilTagSystem withAmbiguityLessThan(double ambiguity) {
        maxAmbiguity = ambiguity;
        return this;
    }

    public PhotonAprilTagSystem withDefaultAmbiguity() {
        return withAmbiguityLessThan(DEFAULT_ACCEPTABLE_AMBIGUITY);
    }
}
