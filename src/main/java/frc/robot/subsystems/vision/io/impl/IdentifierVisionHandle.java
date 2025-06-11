package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public class IdentifierVisionHandle<I> implements VisionHandle<I> {
    protected final VisionCameraID cameraID;
    private final VisionProcessorPipelineManager<I> processorPipelineManager;
    private final VisionCameraHardware<I> cameraHardware;

    private VisionProcessorType<? extends VisionProcessor> currentProcessorType;
    private I queuedID;

    public IdentifierVisionHandle(VisionCameraID cameraID,
                                  VisionCameraHardware<I> cameraHardware,
                                  VisionProcessorPipelineManager<I> processorPipelineManager,
                                  I initialPipelineID) {
        this.processorPipelineManager = processorPipelineManager;
        this.cameraHardware = cameraHardware;
        this.cameraID = cameraID;
        this.currentProcessorType = processorPipelineManager.getVisionProcessorType(initialPipelineID).orElse(null);
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
        if (queuedID != null && cameraHardware.getPipelineID().equals(queuedID)) {
            currentProcessorType = processorPipelineManager.getVisionProcessorType(queuedID).orElse(currentProcessorType);
            queuedID = null;
        }

        return currentProcessorType;
    }

    @Override
    public boolean switchToPipeline(I pipelineID) {
        Optional<VisionProcessorType<? extends VisionProcessor>> pipelineProcessorType = processorPipelineManager.getVisionProcessorType(pipelineID);

        if (pipelineProcessorType.isPresent()) {
            currentProcessorType = pipelineProcessorType.get();
            cameraHardware.setPipelineID(pipelineID);
            return true;
        }

        return false;
    }
}
