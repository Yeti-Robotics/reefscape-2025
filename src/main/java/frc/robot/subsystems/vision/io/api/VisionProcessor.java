package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.VisionSubsystem;

// Aim to use pattern matching to switch between different vision processors
// once WPILib supported Java 21, for now we use Java 17 preview build
// See https://openjdk.org/jeps/441
public sealed interface VisionProcessor permits VisionAprilTagProcessor, VisionNNProcessor {
    /**
     * Called periodically in {@link VisionSubsystem}
     */
    default void visionPeriodic() {}
}
