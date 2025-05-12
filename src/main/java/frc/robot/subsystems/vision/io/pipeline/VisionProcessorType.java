package frc.robot.subsystems.vision.io.pipeline;

import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessor;

public class VisionProcessorType<T extends VisionProcessor> {
    public static final VisionProcessorType<VisionAprilTagProcessor> APRILTAG =
            new VisionProcessorType<>(VisionAprilTagProcessor.class);
    public static final VisionProcessorType<VisionNNProcessor> NN = new VisionProcessorType<>(VisionNNProcessor.class);

    final Class<T> clazz;

    VisionProcessorType(Class<T> clazz) {
        this.clazz = clazz;
    }
}
