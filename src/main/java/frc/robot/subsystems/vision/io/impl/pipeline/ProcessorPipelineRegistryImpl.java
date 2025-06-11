package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.VisionSettings;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.*;

public class ProcessorPipelineRegistryImpl<I> implements ProcessorPipelineRegistry<I>, VisionProcessorPipelineManager<I> {
    private final Map<I, VisionProcessorType<? extends VisionProcessor>> idToProcessors = new HashMap<>();
    private final Map<VisionProcessorType<? extends VisionProcessor>, ? super VisionProcessor> processorTypeMap = new HashMap<>();
    private final Map<I, VisionSettings<?>> pipelineToSettings = new HashMap<>();

    @SuppressWarnings("unchecked")
    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType) {
        return Optional.ofNullable((T) processorTypeMap.get(visionProcessorType));
    }

    @Override
    public Optional<VisionProcessorType<? extends VisionProcessor>> getVisionProcessorType(I pipelineID) {
        return Optional.ofNullable(idToProcessors.get(pipelineID));
    }

    @Override
    public <T extends VisionProcessor> VisionSettings<T> getSettings(I pipelineID) {
        return null;
    }

    @Override
    public int pipelineCount() {
        return idToProcessors.size();
    }

    @Override
    public <T extends VisionProcessor> void addPipelineID(I identifier, VisionProcessorType<T> visionProcessorType) {
        if (!processorTypeMap.containsKey(visionProcessorType)) {
            throw new IllegalArgumentException("Unknown VisionProcessorType: " + visionProcessorType);
        } else if (idToProcessors.containsKey(identifier)) {
            throw new IllegalArgumentException("Duplicate Pipeline ID: " + identifier);
        }

        idToProcessors.put(identifier, visionProcessorType);
    }

    @SuppressWarnings("unchecked")
    @Override
    public <T extends VisionProcessor> void setPipelineSettings(I identifier, VisionProcessorType<T> visionProcessorType, VisionSettings<T> visionSettings) {
        if (idToProcessors.get(identifier) == visionProcessorType) {
            pipelineToSettings.put(identifier, visionSettings);

            if (!pipelineToSettings.containsKey(identifier)) {
                T processor = (T) processorTypeMap.get(visionProcessorType);

                visionSettings.applySettings(processor);
            }
        }
    }

    @Override
    public <T extends VisionProcessor> void registerProcessor(VisionProcessorType<T> visionProcessorType, T processor) {
        processorTypeMap.put(visionProcessorType, processor);
    }

    @Override
    public <T extends VisionProcessor> boolean supports(VisionProcessorType<T> visionProcessorType) {
        return processorTypeMap.containsKey(visionProcessorType);
    }

    @Override
    public boolean hasPipelineID(I identifier) {
        return idToProcessors.containsKey(identifier);
    }

    @Override
    public VisionProcessorPipelineManager<I> toPipelineManager() {
        return this;
    }
}
