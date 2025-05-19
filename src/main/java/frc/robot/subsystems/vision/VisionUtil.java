package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.SimCameraProperties;

import java.util.Optional;
import java.util.function.Consumer;

public class VisionUtil {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static double getDistanceMeters(Transform3d transform) {
        return transform.getTranslation().getNorm();
    }

    public static Optional<Pose3d> getPoseForTag(int fiducialID) {
        return APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialID);
    }

    public static Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(
                transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }
}
