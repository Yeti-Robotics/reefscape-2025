package frc.robot.subsystems.vision.io.api.processor;

import frc.robot.subsystems.vision.data.apriltag.VisionAprilTag3D;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;

import java.util.List;
import java.util.Optional;

public interface VisionAprilTag3DProcessor extends VisionProcessor {
    List<VisionRobotPose> getRobotPoseObservation();

    List<VisionAprilTag3D> getLatestAprilTagObservations();

    Optional<VisionAprilTag3D> getBestAprilTagObservation();

    VisionAprilTagSettings getSettings();
}
