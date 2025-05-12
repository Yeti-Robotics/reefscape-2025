package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandle;
import org.photonvision.PhotonCamera;

/**
 * Specialized vision handle for PhotonVision cameras.
 * This class provides PhotonVision-specific functionality for switching pipelines.
 */
public class PhotonVisionHandle extends AbstractIndexedVisionHandle {
    private final PhotonCamera camera;

    /**
     * Creates a PhotonVision vision handle for a predefined camera ID.
     * 
     * @param camera The camera ID
     */
    public PhotonVisionHandle(VisionCameraID cameraID, PhotonCamera camera, VisionProcessor[] visionProcessors) {
        super(cameraID, visionProcessors);
        this.camera = camera;
    }

    @Override
    protected void switchPipeline(int pipelineIdentifier) {
        camera.setPipelineIndex(pipelineIdentifier);
    }

    /**
     * Gets the PhotonCamera instance for this handle.
     * 
     * @return The PhotonCamera instance
     */
    public PhotonCamera getCamera() {
        return camera;
    }
}