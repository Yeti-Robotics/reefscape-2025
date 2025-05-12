package frc.robot.subsystems.vision.processor.impl.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.processor.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.processor.VisionAprilTagSettings;
import frc.robot.subsystems.vision.processor.impl.limelight.util.LimelightDataParsingHelper;
import frc.robot.subsystems.vision.processor.impl.limelight.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionAprilTagLimelight implements VisionAprilTagProcessor {
    private final String limelightName;
    private final VisionAprilTagSettings.VisionAprilTagMode mode;
    private final Supplier<Rotation2d> drivetrainRotation;

    private List<VisionAprilTag> aprilTagObservations;
    private final List<VisionRobotPose> robotPoseObservation = Arrays.asList(new VisionRobotPose[1]);

    public VisionAprilTagLimelight(
            String limelightName,
            Transform3d robotToCameraTransform,
            Supplier<Rotation2d> drivetrainRotation,
            VisionAprilTagSettings.VisionAprilTagMode mode) {
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
    public List<VisionAprilTag> getLatestAprilTagObservations() {
        return aprilTagObservations;
    }

    @Override
    public Optional<VisionAprilTag> getBestAprilTagObservation() {
        TimestampedDoubleArray tagEntry = LimelightHelpers.getLimelightDoubleArrayEntry(
                        limelightName, "targetpose_robotspace")
                .getAtomic();

        Pose3d tagPose = LimelightHelpers.toPose3D(tagEntry.value);
        if (tagPose == null || tagPose == Pose3d.kZero) return Optional.empty();
        int id = (int) LimelightHelpers.getFiducialID(limelightName);

        return Optional.of(new VisionAprilTag(id, tagPose.toPose2d(), 0, tagEntry.timestamp));
    }

    @Override
    public void visionPeriodic() {
        LimelightHelpers.SetRobotOrientation(
                limelightName, drivetrainRotation.get().getDegrees(), 0, 0, 0, 0, 0);

        if (mode.hasEnabled(VisionAprilTagSettings.VisionAprilTagOptions.ALL_DETECTIONS)) {
            if (aprilTagObservations == null) aprilTagObservations = new ArrayList<>();
            LimelightHelpers.LimelightResults results = LimelightDataParsingHelper.getResults(limelightName);

            if (!results.valid || results.targets_Fiducials.length == 0) return;

            aprilTagObservations.clear();

            for (LimelightHelpers.LimelightTarget_Fiducial fiducial : results.targets_Fiducials) {
                // TODO: test if this is the correct ambiguity value
                aprilTagObservations.add(new VisionAprilTag(
                        (int) fiducial.fiducialID,
                        fiducial.getTargetPose_RobotSpace2D(),
                        0,
                        results.timestamp_LIMELIGHT_publish));
            }
        }
    }
}
