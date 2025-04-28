package frc.robot.subsystems.vision.io;

import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionObservation;
import frc.robot.subsystems.vision.data.VisionRobotPose;

import java.util.List;

public interface VisionAprilTagIO {
    List<VisionObservation<VisionRobotPose>> getRobotPoseObservation();

    List<VisionObservation<VisionAprilTag>> getAprilTagObservations();
}
