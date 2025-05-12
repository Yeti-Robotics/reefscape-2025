package frc.robot.subsystems.vision.processor;

import frc.robot.subsystems.vision.data.VisionNNDetection;

import java.util.List;

public non-sealed interface VisionNNProcessor extends VisionProcessor {
    List<VisionNNDetection> getLatestNNDetections();
}
