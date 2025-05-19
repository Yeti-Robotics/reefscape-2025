package frc.robot.subsystems.vision.io.impl.pipeline;

public interface PipelineManager<T> {
    T getPipeline();
    void setPipeline(T pipeline);
}
