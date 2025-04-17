package frc.robot.subsystems.vision.apriltag.impl.photon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TagConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.AprilTagDetectionHelpers;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PhotonAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private static final double MAX_LIVE_SECONDS = 5;
    private PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;

    private double maxAmbiguity = 0.2;
    private static final double maxTagDistance = 5;
    private static final double maxThetaVarianceDegrees = 10;

    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<AprilTagDetection> bestDetection;

    private double bestDetectionTimestamp;
    private final List<AprilTagPose> poseEstimates = new ArrayList<>();

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
                PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    public boolean poseIsReasonable(Pose2d pose) {
        return
                Math.abs(pose.getRotation().minus(drivetrain.getPigeon2().getRotation2d()).getDegrees()) < maxThetaVarianceDegrees;
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        poseEstimates.clear();

        PhotonTrackedTarget closestTarget = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        double closestTargetTimestamp = 0;

        for (PhotonPipelineResult pipelineResult : results) {
            Optional<MultiTargetPNPResult> multiTargetPNPResult = pipelineResult.getMultiTagResult();

            boolean noMulti = multiTargetPNPResult.isEmpty();

            if (!noMulti) {
                MultiTargetPNPResult multiResult = multiTargetPNPResult.get();

                if (multiResult.estimatedPose.ambiguity > maxAmbiguity) {
                    pipelineResult.multitagResult = Optional.empty();
                    noMulti = true;
                }
            }

            if (noMulti) {
                pipelineResult.getTargets()
                        .removeIf(target -> target.getPoseAmbiguity() > maxAmbiguity || AprilTagDetectionHelpers.getDetectionDistance(target.bestCameraToTarget) > maxTagDistance);
            }

            if (pipelineResult.hasTargets()) {
                for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
                    double tagDistance = AprilTagDetectionHelpers.getDetectionDistance(target.bestCameraToTarget);

                    if (tagDistance < closestDistance && closestTargetTimestamp < pipelineResult.getTimestampSeconds()) {
                        closestDistance = tagDistance;
                        closestTargetTimestamp = pipelineResult.getTimestampSeconds();
                        closestTarget = target;
                    }
                }

                Optional<EstimatedRobotPose> estimatedRobotPoseOptional = photonPoseEstimator.update(pipelineResult);

                if (estimatedRobotPoseOptional.isPresent()) {
                    EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOptional.get();
                    Pose2d robotPose = estimatedRobotPose.estimatedPose.toPose2d();

                    if (poseIsReasonable(robotPose)) {
                        poseEstimates.add(
                                new AprilTagPose(
                                        estimatedRobotPose.estimatedPose.toPose2d(),
                                        estimatedRobotPose.targetsUsed.size(),
                                        estimatedRobotPose.timestampSeconds
                                )
                        );
                    }
                }
            }
        }

        Optional<AprilTagDetection> bestDetectionOpt = mapToDetection(closestTarget);

        if (bestDetectionOpt.isPresent()) {
            bestDetection = bestDetectionOpt;
            bestDetectionTimestamp = closestTargetTimestamp;
        } else if (bestDetectionTimestamp > MAX_LIVE_SECONDS) {
            bestDetection = Optional.empty();
        }
    }

    public void setCamera(PhotonCamera camera) {
        this.camera = camera;
    }

    private Optional<AprilTagDetection> mapToDetection(PhotonTrackedTarget target) {
        if (target == null || target.getPoseAmbiguity() > maxAmbiguity) return Optional.empty();

        Optional<Pose3d> optAprilTagPose = TagConstants.getTagPose(target.fiducialId);

        if (optAprilTagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d aprilTagPose = optAprilTagPose.get();

        Pose3d robotPose =
                PhotonUtils.estimateFieldToRobotAprilTag(
                        target.bestCameraToTarget, aprilTagPose, cameraTransform.inverse());

        Pose3d targetPose =
                Pose3d.kZero.transformBy(cameraTransform).transformBy(target.bestCameraToTarget);

        return Optional.of(
                new AprilTagDetection(
                        target.getFiducialId(),
                        robotPose.toPose2d(),
                        targetPose.toPose2d(),
                        target.getPoseAmbiguity()));
    }

    @Override
    public Optional<AprilTagResults> getResults() {
        // my favoritest method implementation ever!
        // but seriously, we don't use this so it's kinda a waste of rio CPU
        throw new UnsupportedOperationException("Not implemented.");
    }

    @Override
    public List<AprilTagPose> getEstimatedPose() {
        return poseEstimates;
    }

    @Override
    public Optional<AprilTagDetection> getBestDetection() {
        return bestDetection;
    }

    public PhotonAprilTagSystem withAmbiguityLessThan(double ambiguity) {
        maxAmbiguity = ambiguity;
        return this;
    }
}
