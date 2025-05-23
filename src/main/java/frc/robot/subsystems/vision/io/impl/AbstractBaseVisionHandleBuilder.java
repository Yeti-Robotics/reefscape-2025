package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.api.processor.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Abstract base class for vision handle builders.
 * This class provides common functionality for all vision handle builders.
 */
public abstract class AbstractBaseVisionHandleBuilder<H extends VisionHandle, B extends VisionHandleBuilder, I> implements VisionHandleBuilder {
    public static class VisionProcessorData<T extends VisionProcessor, I> {
        protected final T processor;
        protected I pipelineID;

        protected VisionProcessorData(T processor, I pipelineID) {
            this.processor = processor;
            this.pipelineID = pipelineID;
        }

        protected VisionProcessorData(T processor) {
            this.processor = processor;
            this.pipelineID = null;
        }

        protected I getPipelineID() {
            return pipelineID;
        }

        protected void setPipelineID(I pipelineID) {
            this.pipelineID = pipelineID;
        }
    }

    protected final VisionCameraID<H, B> cameraID;
    protected final Supplier<Rotation2d> drivetrainRotation;
    protected final Transform3d robotToCameraTransform;
    protected VisionProcessorType<? extends VisionProcessor> mainProcessorType;
    protected final Map<VisionProcessorType<? extends VisionProcessor>, VisionProcessorData<? extends VisionProcessor, I>> registeredProcessors = new HashMap<>();

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public AbstractBaseVisionHandleBuilder(
            VisionCameraID<H, B> cameraID,
            Supplier<Rotation2d> drivetrainRotation,
            Transform3d robotToCameraTransform) {
        this.cameraID = cameraID;
        this.drivetrainRotation = drivetrainRotation;
        this.robotToCameraTransform = robotToCameraTransform;
    }

    /**
     * Adds a custom processor to the vision handle.
     *
     * @param <T>           The type of processor
     * @param processorType The processor type
     * @param processor     The processor instance
     * @return This builder for chaining
     */
    protected <T extends VisionProcessor> B addProcessor(
            VisionProcessorType<T> processorType, T processor, I identifier) {
        if (mainProcessorType == null) {
            mainProcessorType = processorType;
        }

        if (!registeredProcessors.containsKey(processorType)) {
            registeredProcessors.put(processorType, new VisionProcessorData<>(processor, identifier));
        }

        return getThis();
    }

    public <T extends VisionProcessor> B setPipelineID(
            VisionProcessorType<T> processorType, I pipelineID) {
        registeredProcessors.computeIfPresent(processorType,
                (_k, v) -> {
                    v.setPipelineID(pipelineID);
                    return v;
                });

        return getThis();
    }

    public <T extends VisionProcessor> B setDefaultProcessor(VisionProcessorType<T> processorType) {
        mainProcessorType = processorType;
        return getThis();
    }

    protected VisionProcessorManager makeProcessorManager() {
        if (registeredProcessors.isEmpty()) {
            throw new IllegalStateException("No vision processors were added to the vision handle builder!");
        } else if (registeredProcessors.size() == 1) {
            Map.Entry<VisionProcessorType<? extends VisionProcessor>, VisionProcessorData<? extends VisionProcessor, I>> singleProcessorData =
                    registeredProcessors.entrySet().iterator().next();

            return new DefaultSingleProcessorManager<>(singleProcessorData.getKey(), singleProcessorData.getValue().processor);
        } else {
            return new DefaultMultiProcessorManager<>(
                    registeredProcessors,
                    createPipelineManager(),
                    mainProcessorType
            );
        }
    }


    @Override
    public H build() {
        return buildWithManager(makeProcessorManager());
    }

    protected abstract B getThis();

    protected abstract PipelineManager<I> createPipelineManager();

    protected abstract H buildWithManager(VisionProcessorManager processorManager);

    /**
     * Creates an AprilTag processor for the current camera type.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return The created processor
     */
    protected abstract VisionAprilTag3DProcessor createAprilTagProcessor(
            VisionAprilTagSettingsConfigurator.VisionAprilTagSettings aprilTagMode);

    /**
     * Creates a neural network processor for the current camera type.
     *
     * @param classNames The class names for neural network detection
     * @return The created processor
     */
    protected abstract VisionNNProcessor createNNProcessor(String[] classNames);
}
