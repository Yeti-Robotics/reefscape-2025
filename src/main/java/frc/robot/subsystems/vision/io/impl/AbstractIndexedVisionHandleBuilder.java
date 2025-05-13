package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Abstract base class for vision handle builders.
 * This class provides common functionality for all vision handle builders.
 */
public abstract class AbstractIndexedVisionHandleBuilder implements VisionHandleBuilder {
    protected final VisionCameraID cameraID;
    protected final Supplier<Rotation2d> drivetrainRotation;
    protected final Transform3d robotToCameraTransform;
    protected final List<VisionProcessor> visionProcessors = new ArrayList<>();
    protected boolean hasProcessor = false;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public AbstractIndexedVisionHandleBuilder(
            VisionCameraID cameraID,
            Supplier<Rotation2d> drivetrainRotation,
            Transform3d robotToCameraTransform) {
        this.cameraID = cameraID;
        this.drivetrainRotation = drivetrainRotation;
        this.robotToCameraTransform = robotToCameraTransform;
    }

    /**
     * Adds an AprilTag processor to the vision handle.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return This builder for chaining
     */
    public AbstractIndexedVisionHandleBuilder addAprilTagProcessor(
            AprilTagVisionSettings aprilTagMode) {
        visionProcessors.add(createAprilTagProcessor(aprilTagMode.settings()));
        return this;
    }

    /**
     * Adds an AprilTag processor to the vision handle.
     * This uses the default settings specified in {@link AprilTagVisionSettings#defaultSettings()}
     * @return This builder for chaining
     */
    public AbstractIndexedVisionHandleBuilder addAprilTagProcessor() {
        visionProcessors.add(createAprilTagProcessor(AprilTagVisionSettings.defaultSettings().settings()));
        return this;
    }

    /**
     * Adds a neural network processor to the vision handle.
     *
     * @param classNames The class names for neural network detection
     * @return This builder for chaining
     */
    public AbstractIndexedVisionHandleBuilder addNNProcessor(String[] classNames) {
        visionProcessors.add(createNNProcessor(classNames));
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
    @Override
    public <T extends VisionProcessor> AbstractIndexedVisionHandleBuilder addProcessor(
            VisionProcessorType<T> processorType, T processor) {
        visionProcessors.add(processor);
        return this;
    }

    /**
     * Creates an AprilTag processor for the current camera type.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return The created processor
     */
    protected abstract VisionAprilTagProcessor createAprilTagProcessor(
            AprilTagVisionSettings.AprilTagVisionMode aprilTagMode);

    /**
     * Creates a neural network processor for the current camera type.
     *
     * @param classNames The class names for neural network detection
     * @return The created processor
     */
    protected abstract VisionNNProcessor createNNProcessor(String[] classNames);
}
