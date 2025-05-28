package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.api.processor.*;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineProcessorRegistry;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineProcessorRegistryImpl;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Abstract base class for vision handle builders.
 * This class provides common functionality for all vision handle builders.
 */
public abstract class AbstractBaseVisionHandleBuilder<H extends VisionHandle, B extends VisionHandleBuilder, I> implements VisionHandleBuilder {
    protected final VisionCameraID cameraID;
    protected final Supplier<Rotation2d> drivetrainRotation;
    protected final Transform3d robotToCameraTransform;
    protected VisionProcessorType<? extends VisionProcessor> mainProcessorType;
    protected final PipelineProcessorRegistry<I> registeredProcessors = new PipelineProcessorRegistryImpl<>();

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

        registeredProcessors.addPipeline(processorType, identifier, processor);

        return getThis();
    }

    public B addAprilTagProcessor(I pipelineID, VisionAprilTagSettings settings) {
        return addProcessor(VisionProcessorType.APRILTAG_3D, createAprilTagProcessor(settings), pipelineID);
    }

    public B addNNProcessor(I pipelineID, String[] classNames) {
        return addProcessor(VisionProcessorType.NN, createNNProcessor(classNames), pipelineID);
    }

    public <T extends VisionProcessor> B setDefaultProcessor(VisionProcessorType<T> processorType) {
        if (processorType != null && registeredProcessors.hasProcessor(processorType)) {
            mainProcessorType = processorType;
        }

        return getThis();
    }

    protected VisionProcessorManager createProcessorManager() {
        if (registeredProcessors.pipelineCount() == 0) {
            throw new IllegalStateException("No vision processors were added to the vision handle builder!");
        } else if (registeredProcessors.pipelineCount() == 1) {
            Optional<? extends VisionProcessor> visionProcessor = registeredProcessors.getProcessor(mainProcessorType);

            if (visionProcessor.isEmpty()) {
                throw new IllegalStateException("Processor for " + mainProcessorType + " is not registered!");
            }

            return new DefaultSingleProcessorManager<>(mainProcessorType, visionProcessor.get());
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
        return buildWithManager(createProcessorManager());
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
            VisionAprilTagSettings aprilTagMode);

    /**
     * Creates a neural network processor for the current camera type.
     *
     * @param classNames The class names for neural network detection
     * @return The created processor
     */
    protected abstract VisionNNProcessor createNNProcessor(String[] classNames);
}
