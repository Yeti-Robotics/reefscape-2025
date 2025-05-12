package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionUtil;
import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.io.api.AprilTagVisionSettings;
import frc.robot.subsystems.vision.io.api.AprilTagVisionSettings.AprilTagVisionFeatures;
import frc.robot.subsystems.vision.io.api.AprilTagVisionSettings.AprilTagVisionMode;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class PhotonVisionAprilTag implements VisionAprilTagProcessor {
    private static final double MAX_APRILTAG_AMBIGUITY = 0.2;
    private static final double MAX_ALLOWABLE_DETECTION_DISTANCE_METERS = 5;
    private static final double MAX_THETA_VARIANCE_DEGREES = 10;

    private final PhotonCamera camera;
    private final Supplier<Rotation2d> drivetrainRotation;

    private final List<VisionRobotPose> poseEstimates = new ArrayList<>();
    private final List<VisionAprilTag> aprilTagEstimates = new ArrayList<>();
    private VisionAprilTag bestAprilTagEstimate = null;

    private final Transform3d robotToCameraTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagVisionSettings.AprilTagVisionMode mode;

    public PhotonVisionAprilTag(
            PhotonCamera camera,
            Transform3d robotToCameraTransform,
            Supplier<Rotation2d> drivetrainRotation,
            AprilTagVisionSettings.AprilTagVisionMode mode) {
        this.camera = camera;
        this.robotToCameraTransform = robotToCameraTransform;
        this.poseEstimator = new PhotonPoseEstimator(
                VisionUtil.APRIL_TAG_FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraTransform);

        this.drivetrainRotation = drivetrainRotation;
        this.mode = mode;

        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    private static double getDistanceMeters(Transform3d transform) {
        return transform.getTranslation().getNorm();
    }

    private static boolean poseIsReasonable(Rotation2d rotation, Pose2d pose) {
        return Math.abs(pose.getRotation().minus(rotation).getDegrees()) < MAX_THETA_VARIANCE_DEGREES;
    }

    private static boolean isAmbiguousTarget(PhotonTrackedTarget target) {
        return target.poseAmbiguity > MAX_APRILTAG_AMBIGUITY
                || getDistanceMeters(target.bestCameraToTarget) > MAX_ALLOWABLE_DETECTION_DISTANCE_METERS;
    }

    @Override
    public void visionPeriodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty())
            return;

        PhotonPipelineResult latestResult = results.get(0);
        aprilTagEstimates.clear();
        poseEstimates.clear();

        for (PhotonPipelineResult pipelineResult : results) {
            if (latestResult.getTimestampSeconds() < pipelineResult.getTimestampSeconds()) {
                latestResult = pipelineResult;
            }

            if (mode.hasEnabled(AprilTagVisionFeatures.LOCALIZATION)) {
                Optional<MultiTargetPNPResult> multiTargetPNPResult = pipelineResult.getMultiTagResult();

                boolean ambiguousMulti = multiTargetPNPResult.isEmpty()
                        || multiTargetPNPResult.get().estimatedPose.ambiguity > MAX_APRILTAG_AMBIGUITY;

                if (ambiguousMulti) {
                    pipelineResult.multitagResult = Optional.empty();

                    pipelineResult.getTargets().removeIf(PhotonVisionAprilTag::isAmbiguousTarget);
                }

                if (pipelineResult.hasTargets()) {
                    Optional<EstimatedRobotPose> estimatedRobotPoseOptional = poseEstimator.update(pipelineResult);

                    if (estimatedRobotPoseOptional.isPresent()) {
                        EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOptional.get();
                        Pose2d robotPose = estimatedRobotPose.estimatedPose.toPose2d();

                        if (poseIsReasonable(drivetrainRotation.get(), robotPose)) {
                            poseEstimates.add(new VisionRobotPose(
                                    robotPose,
                                    mapToTagIds(estimatedRobotPose.targetsUsed),
                                    estimatedRobotPose.timestampSeconds));
                        }
                    }
                }

                if (mode.hasEnabled(AprilTagVisionSettings.AprilTagVisionFeatures.BEST_DETECTION)) {
                    PhotonTrackedTarget bestTarget = getBestTarget(pipelineResult);

                    if (bestTarget != null) {
                        bestAprilTagEstimate = mapToVisionAprilTag(pipelineResult, bestTarget);
                    }
                }
            }
        }

        if (mode.hasEnabled(AprilTagVisionSettings.AprilTagVisionFeatures.ALL_DETECTIONS)) {
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                if (target.fiducialId == -1)
                    continue;

                aprilTagEstimates.add(mapToVisionAprilTag(latestResult, target));
            }
        }
    }

    private static int[] mapToTagIds(List<PhotonTrackedTarget> targets) {
        int[] tagIds = new int[targets.size()];
        for (int i = 0; i < tagIds.length; i++) {
            tagIds[i] = targets.get(i).fiducialId;
        }
        return tagIds;
    }

    private VisionAprilTag mapToVisionAprilTag(PhotonPipelineResult pipelineResult, PhotonTrackedTarget target) {
        Pose3d robotToTargetPose = new Pose3d().transformBy(robotToCameraTransform)
                .transformBy(target.bestCameraToTarget);

        return new VisionAprilTag(
                target.fiducialId,
                robotToTargetPose.toPose2d(),
                target.poseAmbiguity,
                pipelineResult.getTimestampSeconds());
    }

    private PhotonTrackedTarget getBestTarget(PhotonPipelineResult pipelineResult) {
        return pipelineResult.getBestTarget(); // TODO: replace with something more robust
    }

    @Override
    public List<VisionRobotPose> getRobotPoseObservation() {
        return poseEstimates;
    }

    @Override
    public List<VisionAprilTag> getLatestAprilTagObservations() {
        return aprilTagEstimates;
    }

    @Override
    public Optional<VisionAprilTag> getBestAprilTagObservation() {
        return Optional.ofNullable(bestAprilTagEstimate);
    }

    @Override
    public AprilTagVisionMode getSettings() {
        return mode;
    }
}
