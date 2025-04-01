package frc.robot.subsystems.vision.util;

import frc.robot.subsystems.vision.apriltag.AprilTagDetection;

public class AprilTagDetectionHelpers {
    public static double getDetectionDistance(AprilTagDetection target) {
        return target.getRobotToTargetPose().getTranslation().getNorm();
    }
}
