package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;
import org.photonvision.PhotonCamera;

public class PhotonPipelineManager implements PipelineManager<Integer> {
    private final PhotonCamera photonCamera;

    public PhotonPipelineManager(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
    }

    @Override
    public Integer getPipeline() {
        return photonCamera.getPipelineIndex();
    }

    @Override
    public void setPipeline(Integer pipeline) {
        photonCamera.setPipelineIndex(pipeline);
    }
}
