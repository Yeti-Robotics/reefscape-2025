package frc.robot.subsystems.vision.io.api.processor;

import frc.robot.subsystems.vision.VisionSubsystem;


public interface VisionProcessor {
    /**
     * Called periodically in {@link VisionSubsystem}
     */
    default void visionPeriodic() {
    }

    double latencyMs();
}
