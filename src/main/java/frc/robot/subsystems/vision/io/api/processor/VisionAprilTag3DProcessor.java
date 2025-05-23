package frc.robot.subsystems.vision.io.api.processor;

import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTagTracker;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;

import java.util.List;

public interface VisionAprilTag3DProcessor extends VisionProcessor {
    List<VisionRobotPose> getRobotPoseObservation();

    VisionAprilTagTracker getAprilTags();

    VisionAprilTagSettings getSettings();
}
