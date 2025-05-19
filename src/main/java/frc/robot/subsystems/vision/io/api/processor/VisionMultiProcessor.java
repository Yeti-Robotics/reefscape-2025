package frc.robot.subsystems.vision.io.api.processor;

import frc.robot.subsystems.vision.io.api.VisionProcessorType;

import java.util.Collection;
import java.util.Optional;

public interface VisionMultiProcessor extends VisionProcessor {
    Collection<VisionProcessorType<? extends VisionProcessor>> getProcessorTypes();
    <T extends VisionProcessor> Optional<T> getSubProcessor(VisionProcessorType<T> processorType);
}
