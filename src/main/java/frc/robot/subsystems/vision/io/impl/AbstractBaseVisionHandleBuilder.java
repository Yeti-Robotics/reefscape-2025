package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionHandleBuilder;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.impl.pipeline.ProcessorPipelineRegistry;
import frc.robot.subsystems.vision.io.impl.pipeline.ProcessorPipelineRegistryImpl;
import frc.robot.subsystems.vision.io.impl.pipeline.SingleProcessorPipelineManager;

import javax.swing.text.html.Option;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Abstract base class for vision handle builders.
 * This class provides common functionality for all vision handle builders.
 */
public abstract class AbstractBaseVisionHandleBuilder<H extends VisionCameraHardware<I>, B extends VisionHandleBuilder, I> implements VisionHandleBuilder {
    protected final VisionCameraID cameraID;
    protected final Supplier<Rotation2d> drivetrainRotation;
    protected final Transform3d robotToCameraTransform;
    protected final ProcessorPipelineRegistry<I> registeredProcessors;

    protected VisionProcessorType<? extends VisionProcessor> mainProcessorType;

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
        this.registeredProcessors = createProcessorPipelineRegistry();
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

    public B addAprilTagProcessor(I identifier, VisionAprilTagSettings settings) {
        return addProcessor(VisionProcessorType.APRILTAG_3D, createAprilTagProcessor(settings), identifier);
    }

    public B addNNProcessor(I identifier, String[] classNames) {
        return addProcessor(VisionProcessorType.NN, createNNProcessor(classNames), identifier);
    }

    public <T extends VisionProcessor> B setDefaultProcessor(VisionProcessorType<T> processorType) {
        if (registeredProcessors.getPipelineID(processorType).isPresent()) {
            mainProcessorType = processorType;
        }

        return getThis();
    }

    @SuppressWarnings("unchecked")
    private <T extends VisionProcessor> SingleProcessorPipelineManager<T, I> singleProcessorPipelineManager() {
        VisionProcessorType<T> processorType = (VisionProcessorType<T>) mainProcessorType;
        Optional<T> processor = registeredProcessors.getProcessor(processorType);
        Optional<I> pipelineID = registeredProcessors.getPipelineID(mainProcessorType);

        if (processor.isPresent() && pipelineID.isPresent()) {
            return new SingleProcessorPipelineManager<>(processorType, processor.get(), pipelineID.get());
        }

        throw new IllegalStateException("No processor pipeline registered for " + processorType);
    }

    @Override
    public VisionHandle build() {
        int pipelineCount = registeredProcessors.pipelineCount();

        if (pipelineCount > 0) {
            return new IdentifierVisionHandle<>(cameraID, createCameraHardware(),
                        pipelineCount == 1 ? singleProcessorPipelineManager() : registeredProcessors.toPipelineManager()
                    , mainProcessorType);
        }

        throw new IllegalStateException("No vision processors were added to the vision handle builder!");
    }

    protected ProcessorPipelineRegistry<I> createProcessorPipelineRegistry() {
        return new ProcessorPipelineRegistryImpl<>();
    }

    protected abstract B getThis();

    protected abstract H createCameraHardware();

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
