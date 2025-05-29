package frc.robot.subsystems.vision.io.api.processor;

import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;

public class VisionProcessorType<T extends VisionProcessor> {
    public static final VisionProcessorType<VisionAprilTag3DProcessor> APRILTAG_3D =
            new VisionProcessorType<>(VisionAprilTag3DProcessor.class);
    public static final VisionProcessorType<VisionNNProcessor> NN = new VisionProcessorType<>(VisionNNProcessor.class);

    public final Class<T> clazz;

    VisionProcessorType(Class<T> clazz) {
        this.clazz = clazz;
    }

    @Override
    public String toString() {
        return clazz.getSimpleName();
    }
}
