package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;

import java.util.Optional;

public class VisionUtil {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final double MAX_APRILTAG_AMBIGUITY = 0.2;
    public static final double MAX_ALLOWABLE_DETECTION_DISTANCE_METERS = 5;
    public static final double MAX_THETA_VARIANCE_DEGREES = 10;

    public static double getDistanceMeters(Transform3d transform) {
        return transform.getTranslation().getNorm();
    }

    public static Optional<Pose3d> getPoseForTag(int fiducialID) {
        return APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialID);
    }

    public static Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }

    public static boolean poseIsReasonable(Rotation2d rotation, Pose2d pose) {
        return
                Math.abs(pose.getRotation().minus(rotation).getDegrees()) < MAX_THETA_VARIANCE_DEGREES;
    }
}
