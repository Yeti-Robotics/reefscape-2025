package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.processor.AbstractVisionHandleBuilder;
import frc.robot.subsystems.vision.processor.VisionHandleAbstract;
import frc.robot.subsystems.vision.processor.VisionProcessor;
import frc.robot.subsystems.vision.processor.impl.limelight.VisionHandleBuilderLimelight;
import frc.robot.subsystems.vision.processor.impl.photon.VisionHandleBuilderPhoton;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final Map<VisionCameraID, VisionHandleAbstract<?>> visionHandles = new HashMap<>();
    protected final Supplier<Rotation2d> drivetrainRotation;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        drivetrainRotation = () -> drivetrain.getPigeon2().getRotation2d();
    }

    /**
     * Gets a processor of the specified type for a predefined camera ID.
     * 
     * @param <T> The type of processor to get
     * @param cameraID The camera ID
     * @param processorType The processor type to get
     * @return An Optional containing the processor of the specified type, or empty if no processor exists
     */
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionCameraID cameraID, VisionProcessorType<T> processorType) {
        return Optional.ofNullable(visionHandles.get(cameraID).getProcessor(processorType));
    }

    public AbstractVisionHandleBuilder<?> createCamera(
            VisionCameraID cameraID,
            Transform3d robotToCameraTransform) {
        return switch (cameraID.visionType) {
            case LIMELIGHT_MEGATAG_2 -> new VisionHandleBuilderLimelight(
                    cameraID,
                    drivetrainRotation,
                    robotToCameraTransform, this);
            case PHOTONVISION -> VisionHandleBuilderPhoton.createBuilder(
                    cameraID,
                    drivetrainRotation,
                    robotToCameraTransform, this);
        };
    }

    public void addVisionHandle(VisionHandleAbstract<?> handle) {
        visionHandles.put(handle.cameraID, handle);
    }

    @Override
    public void periodic() {
        // Call visionPeriodic on all processors
        for (VisionHandleAbstract<?> handle : visionHandles.values()) {
            VisionProcessor processor = handle.activeVisionProcessor();
            if (processor != null) {
                processor.visionPeriodic();
            }
        }
    }
}
