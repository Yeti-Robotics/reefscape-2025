package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.impl.AbstractVisionHandle;
import frc.robot.subsystems.vision.io.pipeline.PipelineIdentifier;
import org.photonvision.PhotonCamera;

/**
 * Specialized vision handle for PhotonVision cameras.
 * This class provides PhotonVision-specific functionality for switching pipelines.
 */
public class PhotonVisionHandle extends AbstractVisionHandle<Integer> {
    private final PhotonCamera camera;

    /**
     * Creates a PhotonVision vision handle for a predefined camera ID.
     * 
     * @param camera The camera ID
     */
    public PhotonVisionHandle(VisionCameraID cameraID, PhotonCamera camera) {
        super(cameraID);
        this.camera = camera;
    }

    @Override
    protected void switchPipeline(PipelineIdentifier<Integer> pipelineIdentifier) {
        if (pipelineIdentifier != null) {
            camera.setPipelineIndex(pipelineIdentifier.getIdentifier());
        }
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