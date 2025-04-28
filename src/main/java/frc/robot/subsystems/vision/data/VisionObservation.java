package frc.robot.subsystems.vision.data;

import frc.robot.subsystems.vision.identifier.VisionCamera;

public record VisionObservation<T>(VisionCamera camera, double timestamp, T observation) {
}

