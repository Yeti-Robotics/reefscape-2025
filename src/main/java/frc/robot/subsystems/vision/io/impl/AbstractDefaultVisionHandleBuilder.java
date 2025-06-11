package frc.robot.subsystems.vision.io.impl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;

import java.util.function.Supplier;

public abstract class AbstractDefaultVisionHandleBuilder<H extends VisionCameraHardware<Integer>, B extends AbstractDefaultVisionHandleBuilder<H, B>> extends AbstractBaseVisionHandleBuilder<H, B, Integer> {
    private static final int MAX_PIPELINE_INDEX = 10;
    private int pipelineIndex = 0;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public AbstractDefaultVisionHandleBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        super(cameraID, drivetrainRotation, robotToCameraTransform);
    }


    /**
     * Adds an AprilTag processor to the vision handle.
     *
     * @param aprilTagMode The AprilTag processing mode
     * @return This builder for chaining
     */
    public B addAprilTagProcessor(
            VisionAprilTagSettings aprilTagMode) {
        return addProcessor(VisionProcessorType.APRILTAG_3D, createAprilTagProcessor(aprilTagMode));
    }

    /**
     * Adds an AprilTag processor to the vision handle.
     * This uses the default settings
     *
     * @return This builder for chaining
     */
    public B addAprilTagProcessor() {
        return addAprilTagProcessor(VisionAprilTagSettings.defaultSettings());
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
        addSupportedProcessor(processorType, processor);
        addPipeline(pipelineIndex, processorType);
        return getThis();
    }
}
