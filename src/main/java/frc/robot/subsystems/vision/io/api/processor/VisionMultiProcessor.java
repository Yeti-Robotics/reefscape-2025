package frc.robot.subsystems.vision.io.api.processor;

import java.util.Collection;
import java.util.Optional;

public interface VisionMultiProcessor<I> extends VisionProcessor {
    Collection<I> getProcessorPipelines();
}
