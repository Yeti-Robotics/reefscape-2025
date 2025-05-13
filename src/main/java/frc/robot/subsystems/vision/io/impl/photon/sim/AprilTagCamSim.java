package frc.robot.subsystems.vision.io.impl.photon.sim;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagCamSim {
    private final PhotonCamera cam;
    private final PhotonCameraSim cameraSim;
    private final Transform3d transform;

    public AprilTagCamSim(
            PhotonCamera cam,
            PhotonCameraSim cameraSim,
            Transform3d transform) {
        this.cam = cam;
        this.cameraSim = cameraSim;
        this.transform = transform;
    }

    public Transform3d getTransform() {
        return transform;
    }

    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    public PhotonCamera getCamera() {
        return cam;
    }
}

