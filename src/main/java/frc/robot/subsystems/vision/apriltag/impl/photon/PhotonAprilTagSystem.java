package frc.robot.subsystems.vision.apriltag.impl.photon;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.*;
import frc.robot.subsystems.vision.util.AprilTagDetectionHelpers;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private static final double MAX_LIVE_SECONDS = 5;
    private PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    private AprilTagResults aprilTagResults = new AprilTagResults(0, 0, Collections.emptyList());
    private double maxAmbiguity = 1;
    private AprilTagDetection bestDetection;
    private double bestDetectionTimestamp;

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
                        FieldConstants.APRIL_TAG_FIELD_LAYOUT,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cameraTransform);
        this.drivetrain = commandSwerveDrivetrain;

        photonPoseEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
    }

    @Override
    public void periodic() {
        //        SwerveDrivetrain.SwerveDriveState state = drivetrain.getState();
        //        double timestamp =
        //                state.Timestamp - Utils.getCurrentTimeSeconds() +
        // Timer.getFPGATimestamp();
        //        photonPoseEstimator.addHeadingData(timestamp,
        // drivetrain.getRotation3d().toRotation2d());
        //        //
        photonPoseEstimator.setLastPose(drivetrain.getState().Pose);

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        double latestTimestamp = -1;

        PhotonPipelineResult latestResult = null;

        for (PhotonPipelineResult result : results) {
            estimatedRobotPose =
                    photonPoseEstimator.update(
                            result, camera.getCameraMatrix(), camera.getDistCoeffs());

            if (result.getTimestampSeconds() > latestTimestamp) {
                latestResult = result;
            }
        }

        if (latestResult == null || !latestResult.hasTargets()) return;

        List<AprilTagDetection> detections =
                latestResult.getTargets().stream()
                        .map(this::mapToDetection)
                        .flatMap(Optional::stream)
                        .toList();

        Optional<AprilTagDetection> bestDetectionOpt =
                detections.stream()
                        .min(Comparator.comparing(AprilTagDetectionHelpers::getDetectionDistance));

        if (bestDetectionOpt.isPresent()) {
            bestDetection = bestDetectionOpt.get();
            bestDetectionTimestamp = latestResult.getTimestampSeconds();
        } else if (latestResult.getTimestampSeconds() - bestDetectionTimestamp > MAX_LIVE_SECONDS) {
            bestDetection = null;
        }

        aprilTagResults =
                new AprilTagResults(
                        latestResult.getTimestampSeconds(),
                        latestResult.metadata.getLatencyMillis(),
                        detections);
    }

    public void setCamera(PhotonCamera camera) {
        this.camera = camera;
    }

    private Optional<AprilTagDetection> mapToDetection(PhotonTrackedTarget target) {
        if (target.getPoseAmbiguity() > maxAmbiguity) {
            return Optional.empty();
        }

        Optional<Pose3d> optAprilTagPose =
                FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);

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
        return Optional.ofNullable(bestDetection);
    }

    public PhotonAprilTagSystem withAmbiguityLessThan(double ambiguity) {
        maxAmbiguity = ambiguity;
        return this;
    }
}
