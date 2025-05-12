package frc.robot.subsystems.vision.io.api;

public class VisionProcessorType<T extends VisionProcessor> {
    public static final VisionProcessorType<VisionAprilTagProcessor> APRILTAG =
            new VisionProcessorType<>(VisionAprilTagProcessor.class);
    public static final VisionProcessorType<VisionNNProcessor> NN = new VisionProcessorType<>(VisionNNProcessor.class);

    public final Class<T> clazz;

    VisionProcessorType(Class<T> clazz) {
        this.clazz = clazz;
    }
}
