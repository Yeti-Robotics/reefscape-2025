package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;

public interface VisionSettings<T extends VisionProcessor> {
    void applySettings(T processor);
}
