package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.processor.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.processor.VisionProcessor;

public class VisionProcessorType<T extends VisionProcessor> {
    public static final VisionProcessorType<VisionAprilTagProcessor> APRILTAG =
            new VisionProcessorType<>(VisionAprilTagProcessor.class);
    public static final VisionProcessorType<VisionNNProcessor> NN = new VisionProcessorType<>(VisionNNProcessor.class);

    final Class<T> clazz;

    VisionProcessorType(Class<T> clazz) {
        this.clazz = clazz;
    }
}
