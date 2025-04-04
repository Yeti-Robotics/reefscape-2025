package frc.robot.subsystems.vision.apriltag.impl.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.*;
import frc.robot.subsystems.vision.util.AprilTagDetectionHelpers;
import java.util.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private static final double MAX_LIVE_SECONDS = 5;
    private PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;

    private static final double translationBaseStdev = 0.7;
    private static final double rotationBaseStdev = Math.toRadians(30);

    private double maxAmbiguity = 1;

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
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        poseEstimates.clear();

        double latestTimestamp = -1;
        PhotonPipelineResult latestResult = null;
        PhotonTrackedTarget closestTarget = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        // replace this with a counter-controlled loop if needed
        for (PhotonPipelineResult pipelineResult : results) {
            Optional<EstimatedRobotPose> estimatedRobotPoseOpt =
                    photonPoseEstimator.update(pipelineResult);
            double timestamp = pipelineResult.getTimestampSeconds();

            if (timestamp > latestTimestamp) {
                latestResult = pipelineResult;
                latestTimestamp = timestamp;
            }

            if (estimatedRobotPoseOpt.isPresent()) {
                EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOpt.get();
                Optional<MultiTargetPNPResult> multiTargetPNPResultOptional =
                        pipelineResult.getMultiTagResult();

                double ambiguity = 0, distance = 0;
                int numTags;

                distance /= pipelineResult.targets.size();

                if (multiTargetPNPResultOptional.isPresent()) {
                    MultiTargetPNPResult multiPNPResult = multiTargetPNPResultOptional.get();

                    ambiguity = multiPNPResult.estimatedPose.ambiguity;
                    numTags = multiPNPResult.fiducialIDsUsed.size();
                } else {
                    numTags = estimatedRobotPose.targetsUsed.size();
                }

                if (numTags > 0) {
                    boolean isMultiTag = multiTargetPNPResultOptional.isPresent();

                    for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                        double targetDist =
                                AprilTagDetectionHelpers.getDetectionDistance(
                                        target.bestCameraToTarget);

                        if (pipelineResult == latestResult
                                && (closestTarget == null || targetDist < closestDistance)) {
                            closestTarget = target;
                            closestDistance = targetDist;
                        }

                        distance += target.bestCameraToTarget.getTranslation().getNorm();

                        if (!isMultiTag) {
                            ambiguity += target.getPoseAmbiguity();
                        }
                    }

                    distance /= numTags;

                    if (!isMultiTag) {
                        ambiguity /= numTags;
                    }

                    double scaleFactor =
                            (1 / (1 + Math.pow(distance, 1.5))) * (1 + Math.pow(ambiguity, 1.2));
                    double linearStdDevs = translationBaseStdev * scaleFactor;
                    double angularStdDevs = rotationBaseStdev * scaleFactor;

                    /**
                     * If the below allocation worsens performance, use
                     *
                     * @see AprilTagPose#DEFAULT_STD_DEVS instead
                     */
                    Matrix<N3, N1> stdDevs =
                            VecBuilder.fill(linearStdDevs, linearStdDevs, angularStdDevs);

                    poseEstimates.add(
                            new AprilTagPose(
                                    estimatedRobotPose.estimatedPose.toPose2d(),
                                    numTags,
                                    pipelineResult.getTimestampSeconds(),
                                    stdDevs));
                }
            }
        }

        Optional<AprilTagDetection> bestDetectionOpt = mapToDetection(closestTarget);

        if (bestDetectionOpt.isPresent()) {
            bestDetection = bestDetectionOpt;
            bestDetectionTimestamp = latestTimestamp;
        } else if (latestTimestamp - bestDetectionTimestamp > MAX_LIVE_SECONDS) {
            bestDetection = Optional.empty();
        }
    }

    public void setCamera(PhotonCamera camera) {
        this.camera = camera;
    }

    private Optional<AprilTagDetection> mapToDetection(PhotonTrackedTarget target) {
        if (target == null || target.getPoseAmbiguity() > maxAmbiguity) return Optional.empty();

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
