package frc.robot.subsystems.vision.processor;

import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionRobotPose;

import java.util.List;
import java.util.Optional;

public non-sealed interface VisionAprilTagProcessor extends VisionProcessor {
    List<VisionRobotPose> getRobotPoseObservation();

    List<VisionAprilTag> getLatestAprilTagObservations();

    Optional<VisionAprilTag> getBestAprilTagObservation();
}
