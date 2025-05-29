package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import org.photonvision.PhotonCamera;

public class PhotonHardware implements VisionCameraHardware<Integer> {
    private final PhotonCamera photonCamera;

    public PhotonHardware(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
    }

    @Override
    public boolean isConnected() {
        return photonCamera.isConnected();
    }

    @Override
    public Integer getPipelineID() {
        return photonCamera.getPipelineIndex();
    }

    @Override
    public void setPipelineID(Integer pipelineID) {
        photonCamera.setPipelineIndex(pipelineID);
    }
}
