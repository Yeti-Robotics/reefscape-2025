package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorManager;
import frc.robot.subsystems.vision.io.impl.AbstractVisionHandle;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;
import org.photonvision.PhotonCamera;

public class PhotonVisionHandle extends AbstractVisionHandle {
    private final PhotonCamera photonCamera;

    public PhotonVisionHandle(VisionCameraID cameraID, VisionProcessorManager delegate, PhotonCamera photonCamera) {
        super(cameraID, delegate);
        this.photonCamera = photonCamera;
    }

    @Override
    public boolean isConnected() {
        return photonCamera.isConnected();
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }
}
