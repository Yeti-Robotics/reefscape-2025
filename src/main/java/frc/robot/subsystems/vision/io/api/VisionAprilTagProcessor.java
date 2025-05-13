package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;

import java.util.List;
import java.util.Optional;

public non-sealed interface VisionAprilTagProcessor extends VisionProcessor {
    List<VisionRobotPose> getRobotPoseObservation();

    List<VisionAprilTag> getLatestAprilTagObservations();

    Optional<VisionAprilTag> getBestAprilTagObservation();

    VisionAprilTagSettings getSettings();
}
