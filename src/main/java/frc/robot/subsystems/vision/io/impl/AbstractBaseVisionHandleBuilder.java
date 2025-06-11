package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionHandleBuilder;
import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.impl.pipeline.ProcessorPipelineRegistry;
import frc.robot.subsystems.vision.io.impl.pipeline.ProcessorPipelineRegistryImpl;
import frc.robot.subsystems.vision.io.impl.pipeline.SingleProcessorPipelineManager;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Abstract base class for vision handle builders.
 * This class provides common functionality for all vision handle builders.
 */
public abstract class AbstractBaseVisionHandleBuilder<H extends VisionCameraHardware<I>, B extends VisionHandleBuilder<I>, I> implements VisionHandleBuilder<I> {
    protected final VisionCameraID cameraID;
    protected final Supplier<Rotation2d> drivetrainRotation;
    protected final Transform3d robotToCameraTransform;
    protected final ProcessorPipelineRegistry<I> registeredProcessors = createProcessorPipelineRegistry();

    protected I mainPipeline;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public AbstractBaseVisionHandleBuilder(
            VisionCameraID cameraID,
            Supplier<Rotation2d> drivetrainRotation,
            Transform3d robotToCameraTransform) {
        this.cameraID = cameraID;
        this.drivetrainRotation = drivetrainRotation;
        this.robotToCameraTransform = robotToCameraTransform;
    }

    protected <T extends VisionProcessor> B addPipeline(I pipelineID, VisionProcessorType<T> visionProcessorType) {
        registeredProcessors.addPipelineID(pipelineID, visionProcessorType);

        if (mainPipeline == null) {
            mainPipeline = pipelineID;
        }

        return getThis();
    }

    protected <T extends VisionProcessor> B addSupportedProcessor(VisionProcessorType<T> processorType, T processor) {
        if (!registeredProcessors.supports(processorType)) {
            registeredProcessors.registerProcessor(processorType, processor);
        }

        return getThis();
    }

    public B setDefaultPipeline(I pipelineID) {
        if (registeredProcessors.hasPipelineID(pipelineID)) {
            mainPipeline = pipelineID;
        }

        return getThis();
    }

    @SuppressWarnings("unchecked")
    private <T extends VisionProcessor> VisionProcessorPipelineManager<I> createSingleProcessorPipelineManager() {
        VisionProcessorPipelineManager<I> temp = registeredProcessors.toPipelineManager();

        Optional<VisionProcessorType<? extends VisionProcessor>> vpTypeOpt = temp.getVisionProcessorType(mainPipeline);

        if (vpTypeOpt.isEmpty()) {
            throw new IllegalStateException("No VisionProcessor found for pipeline " + mainPipeline);
        }

        VisionProcessorType<T> vpType = (VisionProcessorType<T>) vpTypeOpt.get();

        Optional<T> vpOpt = temp.getProcessor(vpType);

        if (vpOpt.isEmpty()) {
            throw new IllegalStateException("No VisionProcessor found for pipeline " + vpType);
        }

        return new SingleProcessorPipelineManager<>(vpType, vpOpt.get(), mainPipeline);
    }

    @Override
    public VisionHandle<I> build() {
        int pipelineCount = registeredProcessors.pipelineCount();

        if (pipelineCount < 1) {
            throw new IllegalStateException("No vision processors were added to the vision handle builder!");
        }

        VisionProcessorPipelineManager<I> processorPipelineManager = registeredProcessors.pipelineCount() == 1 ?
                createSingleProcessorPipelineManager() : registeredProcessors.toPipelineManager();

        return new IdentifierVisionHandle<>(cameraID, createCameraHardware(), processorPipelineManager, mainPipeline);
    }

    protected abstract B getThis();

    protected ProcessorPipelineRegistry<I> createProcessorPipelineRegistry() {
        return new ProcessorPipelineRegistryImpl<>();
    }

    protected abstract H createCameraHardware();

    /**
     * Creates an AprilTag processor for the current camera type.
     *
     * @param visionAprilTagSettings The AprilTag processing mode
     * @return The created processor
     */
    protected abstract VisionAprilTag3DProcessor createAprilTagProcessor(
            VisionAprilTagSettings visionAprilTagSettings);

    /**
     * Creates a neural network processor for the current camera type.
     *
     * @param classNames The class names for neural network detection
     * @return The created processor
     */
    protected abstract VisionNNProcessor createNNProcessor(String[] classNames);
}
