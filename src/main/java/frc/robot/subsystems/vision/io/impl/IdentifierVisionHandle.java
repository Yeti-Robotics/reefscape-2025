package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public class IdentifierVisionHandle<I> implements VisionHandle {
    protected final VisionCameraID cameraID;
    private final VisionProcessorPipelineManager<I> processorPipelineManager;
    private final VisionCameraHardware<I> cameraHardware;

    private VisionProcessorType<? extends VisionProcessor> currentProcessorType;

    public IdentifierVisionHandle(VisionCameraID cameraID,
                                  VisionCameraHardware<I> cameraHardware,
                                  VisionProcessorPipelineManager<I> processorPipelineManager,
                                  VisionProcessorType<? extends VisionProcessor> initialProcessorType) {
        this.processorPipelineManager = processorPipelineManager;
        this.cameraHardware = cameraHardware;
        this.cameraID = cameraID;
        this.currentProcessorType = initialProcessorType;
    }

    @Override
    public VisionCameraID identifier() {
        return cameraID;
    }

    @Override
    public boolean isConnected() {
        return cameraHardware.isConnected();
    }

    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType) {
        return processorPipelineManager.getProcessor(visionProcessorType);
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        return currentProcessorType;
    }

    @Override
    public <T extends VisionProcessor> boolean switchToProcessor(VisionProcessorType<T> visionProcessorType) {
        if (currentProcessorType == visionProcessorType) return true;
        if (!processorPipelineManager.hasProcessor(visionProcessorType)) return false;

        Optional<I> identifier = processorPipelineManager.getPipelineID(visionProcessorType);

        if (identifier.isPresent()) {
            I pipelineID = identifier.get();
            cameraHardware.setPipelineID(pipelineID);
            currentProcessorType = visionProcessorType;
            return true;
        }

        return false;
    }
}
