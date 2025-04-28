package frc.robot.subsystems.vision.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionUtil;
import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionObservation;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.identifier.VisionCamera;
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

import static frc.robot.subsystems.vision.VisionUtil.*;

public class VisionAprilTagPhoton extends SubsystemBase implements VisionAprilTagIO {
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;

    private final List<VisionObservation<VisionRobotPose>> poseEstimates = new ArrayList<>();
    private final List<VisionObservation<VisionAprilTag>> aprilTagEstimates = new ArrayList<>();

    private final Transform3d robotToCameraTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final VisionCamera cameraIdentifier;

    public VisionAprilTagPhoton(PhotonCamera camera, Transform3d robotToCameraTransform,
                                CommandSwerveDrivetrain drivetrain, VisionCamera cameraIdentifier) {
        this.camera = camera;
        this.robotToCameraTransform = robotToCameraTransform;
        this.poseEstimator = new PhotonPoseEstimator(
                VisionUtil.APRIL_TAG_FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraTransform
        );
        this.drivetrain = drivetrain;
        this.cameraIdentifier = cameraIdentifier;
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        aprilTagEstimates.clear();
        poseEstimates.clear();

        for (PhotonPipelineResult pipelineResult : results) {
            Optional<MultiTargetPNPResult> multiTargetPNPResult = pipelineResult.getMultiTagResult();

            boolean ambiguousMulti = multiTargetPNPResult.isEmpty() ||
                    multiTargetPNPResult.get().estimatedPose.ambiguity > MAX_APRILTAG_AMBIGUITY;

            if (ambiguousMulti) {
                pipelineResult.multitagResult = Optional.empty();

                pipelineResult.getTargets()
                        .removeIf(target -> target.getPoseAmbiguity() > MAX_APRILTAG_AMBIGUITY ||
                                VisionUtil.getDistanceMeters(target.bestCameraToTarget) > MAX_ALLOWABLE_DETECTION_DISTANCE_METERS);
            }

            if (pipelineResult.hasTargets()) {
                Optional<EstimatedRobotPose> estimatedRobotPoseOptional = poseEstimator.update(pipelineResult);

                if (estimatedRobotPoseOptional.isPresent()) {
                    EstimatedRobotPose estimatedRobotPose = estimatedRobotPoseOptional.get();
                    Pose2d robotPose = estimatedRobotPose.estimatedPose.toPose2d();

                    if (VisionUtil.poseIsReasonable(
                            drivetrain.getPigeon2().getRotation2d(),
                            robotPose)) {
                        poseEstimates.add(
                                new VisionObservation<>(
                                        cameraIdentifier,
                                        estimatedRobotPose.timestampSeconds,
                                        new VisionRobotPose(robotPose, pipelineResult.getTargets().size())
                                )
                        );
                    }
                }

                for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
                    if (target.fiducialId == -1) continue;

                    Pose3d robotToTargetPose = new Pose3d()
                            .transformBy(robotToCameraTransform)
                            .transformBy(target.bestCameraToTarget);

                    aprilTagEstimates.add(new VisionObservation<>(cameraIdentifier, pipelineResult.getTimestampSeconds(),
                            new VisionAprilTag(target.fiducialId, robotToTargetPose.toPose2d(), target.poseAmbiguity)
                    ));
                }
            }
        }
    }

    @Override
    public List<VisionObservation<VisionRobotPose>> getRobotPoseObservation() {
        return poseEstimates;
    }

    @Override
    public List<VisionObservation<VisionAprilTag>> getAprilTagObservations() {
        return aprilTagEstimates;
    }
}
