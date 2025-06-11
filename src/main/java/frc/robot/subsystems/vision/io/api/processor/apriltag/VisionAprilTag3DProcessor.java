package frc.robot.subsystems.vision.io.api.processor.apriltag;

import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTagTracker;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;

import java.util.List;

public interface VisionAprilTag3DProcessor extends VisionProcessor {
    List<VisionRobotPose> getRobotPoseObservation();

    VisionAprilTagTracker getAprilTags();

    VisionAprilTagSettings getSettings();
    void setSettings(VisionAprilTagSettings settings);
}
