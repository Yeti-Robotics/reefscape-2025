package frc.robot.subsystems.vision.processor;

/**
 * Represents a pipeline identifier for a vision processor.
 * This class is designed to be extensible to accommodate different types of pipeline identifiers
 * (e.g., int, String) in the future.
 */
public interface PipelineIdentifier<T> {
    T getIdentifier();
}