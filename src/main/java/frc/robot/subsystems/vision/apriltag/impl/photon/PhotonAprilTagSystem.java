package frc.robot.subsystems.vision.apriltag.impl.photon;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class PhotonAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    private AprilTagResults aprilTagResults = new AprilTagResults(0, 0, Collections.emptyList());
    private double maxAmbiguity = 1;
    private PhotonTrackedTarget currentBestDetection;

    @Logged(name = "TagPoses")
    public List<Pose2d> getTagPoses() {
        return aprilTagResults.getResults().stream()
                .map(AprilTagDetection::getRobotToTargetPose)
                .toList();
    }

    @Logged(name = "Best Detection")
    public Pose2d getBestDetectionPose() {
        var bestDet = getBestDetection();
        return bestDet.map(AprilTagDetection::getRobotToTargetPose).orElse(null);
    }

    @Logged(name = "Best Estimated Pose")
    public Pose2d logBestEstimatedPose() {
        return getEstimatedPose().map(AprilTagPose::getEstimatedRobotPose).orElse(null);
    }

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
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        //
        //        double timestamp =
        //                drivetrain.getState().Timestamp
        //                        - Utils.getCurrentTimeSeconds()
        //                        + Timer.getFPGATimestamp();
        //        photonPoseEstimator.addHeadingData(timestamp,
        // drivetrain.getRotation3d().toRotation2d());
        //
        // photonPoseEstimator.setLastPose(drivetrain.getState().Pose);

        double currentBestDetectionDistance = Double.POSITIVE_INFINITY;
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
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.fiducialId != -1) {
                        double targetNorm = target.bestCameraToTarget.getTranslation().getNorm();

                        if (targetNorm < currentBestDetectionDistance) {
                            currentBestDetection = target;
                            currentBestDetectionDistance = targetNorm;
                        }

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
        return Optional.ofNullable(currentBestDetection).flatMap(this::mapToDetection);
    }

    public PhotonAprilTagSystem withAmbiguityLessThan(double ambiguity) {
        maxAmbiguity = ambiguity;
        return this;
    }
}
