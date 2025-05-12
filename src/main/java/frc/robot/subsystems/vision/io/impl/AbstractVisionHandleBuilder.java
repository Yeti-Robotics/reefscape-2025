package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.pipeline.PipelineIdentifier;
import frc.robot.subsystems.vision.io.pipeline.VisionProcessorType;

import java.util.function.Supplier;

/**
 * Abstract base class for vision handle builders.
 * This class provides common functionality for all vision handle builders.
 */
public abstract class AbstractVisionHandleBuilder<P> {
    protected final String cameraName;
    protected final VisionCameraID.VisionType visionType;
    protected final Supplier<Rotation2d> drivetrainRotation;
    protected final Transform3d robotToCameraTransform;
    protected final VisionHandle<P> handle;
    protected final VisionSubsystem visionSubsystem;
    protected boolean hasProcessor = false;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public AbstractVisionHandleBuilder(
            VisionCameraID cameraID,
            Supplier<Rotation2d> drivetrainRotation,
            Transform3d robotToCameraTransform,
            VisionHandle<P> handle,
            VisionSubsystem visionSubsystem) {
        this.cameraName = cameraID.getCameraName();
        this.visionType = cameraID.visionType;
        this.drivetrainRotation = drivetrainRotation;
        this.robotToCameraTransform = robotToCameraTransform;
        this.handle = handle;
        this.visionSubsystem = visionSubsystem;
    }

    /**
     * Adds an AprilTag processor to the vision handle.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return This builder for chaining
     */
    public AbstractVisionHandleBuilder<P> addAprilTagProcessor(PipelineIdentifier<P> pipelineIdentifier, AprilTagVisionSettings.VisionAprilTagMode aprilTagMode) {
        VisionAprilTagProcessor processor = createAprilTagProcessor(aprilTagMode);
        handle.addProcessor(VisionProcessorType.APRILTAG, pipelineIdentifier, processor);

        if (!hasProcessor) {
            handle.setCurrentProcessor(VisionProcessorType.APRILTAG);
            hasProcessor = true;
        }

        return this;
    }

    /**
     * Adds a neural network processor to the vision handle.
     *
     * @param classNames The class names for neural network detection
     * @return This builder for chaining
     */
    public AbstractVisionHandleBuilder<P> addNNProcessor(PipelineIdentifier<P> pipelineIdentifier, String[] classNames) {
        VisionNNProcessor processor = createNNProcessor(classNames);
        handle.addProcessor(VisionProcessorType.NN, pipelineIdentifier, processor);

        if (!hasProcessor) {
            handle.setCurrentProcessor(VisionProcessorType.NN);
            hasProcessor = true;
        }
        return this;
    }

    /**
     * Adds a custom processor to the vision handle.
     *
     * @param <T>           The type of processor
     * @param processorType The processor type
     * @param processor     The processor instance
     * @return This builder for chaining
     */
    public <T extends VisionProcessor> AbstractVisionHandleBuilder<P> addProcessor(
            VisionProcessorType<T> processorType, PipelineIdentifier<P> pipelineIdentifier, T processor) {
        if (processor != null) {
            handle.addProcessor(processorType, pipelineIdentifier, processor);

            if (!hasProcessor) {
                handle.setCurrentProcessor(processorType);
                hasProcessor = true;
            }
        }
        return this;
    }

    /**
     * Builds the vision handle.
     *
     * @return The built vision handle
     */
    public VisionHandle<P> build() {
        visionSubsystem.addVisionHandle(handle);
        return handle;
    }

    /**
     * Creates an AprilTag processor for the current camera type.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return The created processor
     */
    protected abstract VisionAprilTagProcessor createAprilTagProcessor(AprilTagVisionSettings.VisionAprilTagMode aprilTagMode);

    /**
     * Creates a neural network processor for the current camera type.
     *
     * @param classNames The class names for neural network detection
     * @return The created processor
     */
    protected abstract VisionNNProcessor createNNProcessor(String[] classNames);
}
