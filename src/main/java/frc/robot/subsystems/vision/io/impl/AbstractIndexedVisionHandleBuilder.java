package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator;
import frc.robot.subsystems.vision.io.api.VisionHandleBuilder;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;

import java.util.function.Supplier;

public abstract class AbstractIndexedVisionHandleBuilder<B extends VisionHandleBuilder> extends AbstractBaseVisionHandleBuilder<B, Integer> {
    private int pipelineIndex = 0;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public AbstractIndexedVisionHandleBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        super(cameraID, drivetrainRotation, robotToCameraTransform);
    }


    /**
     * Adds an AprilTag processor to the vision handle.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return This builder for chaining
     */
    public B addAprilTagProcessor(
            VisionAprilTagSettingsConfigurator aprilTagMode) {
        return addProcessor(VisionProcessorType.APRILTAG_3D, createAprilTagProcessor(aprilTagMode.toSettings()));
    }

    /**
     * Adds an AprilTag processor to the vision handle.
     * This uses the default settings specified in {@link VisionAprilTagSettingsConfigurator#defaultSettingsConfig()}
     *
     * @return This builder for chaining
     */
    public B addAprilTagProcessor() {
        return addAprilTagProcessor(VisionAprilTagSettingsConfigurator.defaultSettingsConfig());
    }

    /**
     * Adds a neural network processor to the vision handle.
     *
     * @param classNames The class names for neural network detection
     * @return This builder for chaining
     */
    public B addNNProcessor(String[] classNames) {
        return addProcessor(VisionProcessorType.NN, createNNProcessor(classNames));
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
            VisionProcessorType<T> processorType, T processor) {
        addProcessor(processorType, processor, pipelineIndex);
        pipelineIndex++;
        return getThis();
    }
}
