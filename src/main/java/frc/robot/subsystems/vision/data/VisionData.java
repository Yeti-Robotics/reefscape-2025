package frc.robot.subsystems.vision.data;

import frc.robot.subsystems.vision.VisionCameraID;

public record VisionData<T>(VisionCameraID camera, T data) {}
