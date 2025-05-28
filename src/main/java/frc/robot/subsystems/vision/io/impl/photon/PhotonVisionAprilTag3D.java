package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionUtil;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTagTracker;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings.VisionAprilTagFeature;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
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

public class PhotonVisionAprilTag3D extends AbstractPhotonProcessor implements VisionAprilTag3DProcessor {
    private static final double MAX_APRILTAG_AMBIGUITY = 0.2;
    private static final double MAX_ALLOWABLE_DETECTION_DISTANCE_METERS = 5;
    private static final double MAX_THETA_VARIANCE_DEGREES = 10;

    private final Supplier<Rotation2d> drivetrainRotation;

    private final List<VisionRobotPose> poseEstimates = new ArrayList<>();
    private final VisionAprilTagTracker aprilTagRecorder = new VisionAprilTagTracker();

    private final Transform3d robotToCameraTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final VisionAprilTagSettings mode;

    private PhotonPipelineResult latestResult;

    public PhotonVisionAprilTag3D(
            PhotonCamera camera,
            Transform3d robotToCameraTransform,
            Supplier<Rotation2d> drivetrainRotation,
            VisionAprilTagSettings mode) {
        super(camera);
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
    protected void runPreprocessing() {
        poseEstimates.clear();
    }

    @Override
    protected void processResult(PhotonPipelineResult pipelineResult) {
        if (latestResult == null || latestResult.getTimestampSeconds() < pipelineResult.getTimestampSeconds()) {
            latestResult = pipelineResult;
        }

        if (mode.hasEnabledAll(VisionAprilTagFeature.LOCALIZATION)) {
            Optional<MultiTargetPNPResult> multiTargetPNPResult = pipelineResult.getMultiTagResult();

            boolean ambiguousMulti = multiTargetPNPResult.isEmpty()
                    || multiTargetPNPResult.get().estimatedPose.ambiguity > MAX_APRILTAG_AMBIGUITY;

            if (ambiguousMulti) {
                pipelineResult.multitagResult = Optional.empty();

                pipelineResult.getTargets().removeIf(PhotonVisionAprilTag3D::isAmbiguousTarget);
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

            if (mode.hasEnabledAll(VisionAprilTagFeature.BEST_DETECTION)) {
                PhotonTrackedTarget bestTarget = getBestTarget(pipelineResult);

                if (bestTarget != null) {
                   recordVisionAprilTag(pipelineResult, bestTarget, true);
                }
            }
        }
    }

    @Override
    protected void runPostProcessing() {
        if (mode.hasEnabledAll(VisionAprilTagFeature.ALL_DETECTIONS) && latestResult != null && latestResult.hasTargets()) {
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                recordVisionAprilTag(latestResult, target);
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

    private void recordVisionAprilTag(PhotonPipelineResult pipelineResult, PhotonTrackedTarget target) {
        recordVisionAprilTag(pipelineResult, target, false);
    }

    private void recordVisionAprilTag(PhotonPipelineResult pipelineResult, PhotonTrackedTarget target, boolean isBestDetection) {
        if (target.fiducialId == -1) return;

        Pose3d robotToTargetPose = new Pose3d().transformBy(robotToCameraTransform)
                .transformBy(target.bestCameraToTarget);

        if (isBestDetection) {
            aprilTagRecorder.addBestObservation(
                    target.fiducialId,
                    robotToTargetPose.toPose2d(),
                    target.poseAmbiguity,
                    pipelineResult.getTimestampSeconds()
            );
        } else {
            aprilTagRecorder.addObservation(
                    target.fiducialId,
                    robotToTargetPose.toPose2d(),
                    target.poseAmbiguity,
                    pipelineResult.getTimestampSeconds()
            );
        }
    }

    private PhotonTrackedTarget getBestTarget(PhotonPipelineResult pipelineResult) {
        return pipelineResult.hasTargets() ? pipelineResult.getBestTarget() : null; // TODO: replace with something more robust
    }

    @Override
    public List<VisionRobotPose> getRobotPoseObservation() {
        return poseEstimates;
    }

    @Override
    public VisionAprilTagTracker getAprilTags() {
        return aprilTagRecorder;
    }

    @Override
    public VisionAprilTagSettings getSettings() {
        return mode;
    }
}
