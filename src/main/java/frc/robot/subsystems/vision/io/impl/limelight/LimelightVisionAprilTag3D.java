package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTag3D;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTagRecorder;
import frc.robot.subsystems.vision.io.api.processor.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightDataParsingHelper;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class LimelightVisionAprilTag3D implements VisionAprilTag3DProcessor {
    private final String limelightName;
    private final VisionAprilTagSettings mode;
    private final Supplier<Rotation2d> drivetrainRotation;

    private final VisionAprilTagRecorder aprilTagRecorder = new VisionAprilTagRecorder();
    private final List<VisionRobotPose> robotPoseObservation = Arrays.asList(new VisionRobotPose[1]);
    private double latencyPipeline;

    public LimelightVisionAprilTag3D(
            String limelightName,
            Transform3d robotToCameraTransform,
            Supplier<Rotation2d> drivetrainRotation,
            VisionAprilTagSettings mode) {
        this.limelightName = limelightName;
        this.drivetrainRotation = drivetrainRotation;
        this.mode = mode;

        LimelightDataParsingHelper.getInstance().start();

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                robotToCameraTransform.getX(),
                robotToCameraTransform.getY(),
                robotToCameraTransform.getZ(),
                robotToCameraTransform.getRotation().getX(),
                robotToCameraTransform.getRotation().getY(),
                robotToCameraTransform.getRotation().getZ());
    }

    private static int[] mapToTagIds(LimelightHelpers.RawFiducial[] fiducials) {
        int[] tagIds = new int[fiducials.length];

        for (int i = 0; i < tagIds.length; i++) {
            tagIds[i] = fiducials[i].id;
        }

        return tagIds;
    }

    @Override
    public List<VisionRobotPose> getRobotPoseObservation() {
        LimelightHelpers.PoseEstimate poseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return List.of();
        // TODO: potentially use megatag stddev
        robotPoseObservation.set(0, new VisionRobotPose(
                poseEstimate.pose, mapToTagIds(poseEstimate.rawFiducials), poseEstimate.timestampSeconds));

        return robotPoseObservation;
    }

    @Override
    public VisionAprilTagRecorder getAprilTagRecords() {
        return aprilTagRecorder;
    }

    @Override
    public void visionPeriodic() {
        LimelightHelpers.SetRobotOrientation(
                limelightName, drivetrainRotation.get().getDegrees(), 0, 0, 0, 0, 0);


        if (mode.hasEnabled(VisionAprilTagSettingsConfigurator.VisionAprilTagFeature.BEST_DETECTION)) {
            TimestampedDoubleArray tagEntry = LimelightHelpers.getLimelightDoubleArrayEntry(
                            limelightName, "targetpose_robotspace")
                    .getAtomic();

            Pose3d tagPose = LimelightHelpers.toPose3D(tagEntry.value);
            if (!(tagPose == null || tagPose == Pose3d.kZero)) {
                int id = (int) LimelightHelpers.getFiducialID(limelightName);

                aprilTagRecorder.record(id, tagPose.toPose2d(), 0, tagEntry.timestamp);
                aprilTagRecorder.setBestTag(id);
            }
        }

        if (mode.hasEnabled(VisionAprilTagSettingsConfigurator.VisionAprilTagFeature.ALL_DETECTIONS)) {
            LimelightHelpers.LimelightResults results = LimelightDataParsingHelper.getResults(limelightName);

            if (!results.valid || results.targets_Fiducials.length == 0) return;

            for (LimelightHelpers.LimelightTarget_Fiducial fiducial : results.targets_Fiducials) {
                aprilTagRecorder.record((int) fiducial.fiducialID,
                        fiducial.getTargetPose_RobotSpace2D(),
                        0, results.timestamp_LIMELIGHT_publish
                );
            }

            latencyPipeline = results.latency_pipeline;
        }
    }

    @Override
    public double latencyMs() {
        return latencyPipeline;
    }

    @Override
    public VisionAprilTagSettings getSettings() {
        return mode;
    }
}
