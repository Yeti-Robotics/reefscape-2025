package frc.robot.subsystems.vision.io.api.processor;

import frc.robot.subsystems.vision.data.VisionNNDetection;

import java.util.List;

public interface VisionNNProcessor extends VisionProcessor {
    List<VisionNNDetection> getLatestNNDetections();
}
