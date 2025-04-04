package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagDetectionHelpers {

    public static double getDetectionDistance(Pose2d target) {
        return target.getTranslation().getNorm();
    }

    public static double getDetectionDistance(Transform2d target) {
        return target.getTranslation().getNorm();
    }

    // because apparently you can't convert a 2d transform to a 3d one. ok.
    public static double getDetectionDistance(Transform3d target) {
        return target.getTranslation().getNorm();
    }
}
