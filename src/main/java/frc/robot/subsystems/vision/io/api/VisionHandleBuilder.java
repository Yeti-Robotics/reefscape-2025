package frc.robot.subsystems.vision.io.api;

public interface VisionHandleBuilder {
    <T extends VisionProcessor> VisionHandleBuilder addProcessor(VisionProcessorType<T> processorType, T processor);

    VisionHandle build();
}
