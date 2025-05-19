package frc.robot.subsystems.vision.io.impl.photon.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

public class PhotonVisionAprilTagSimulator {
    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private static final PhotonVisionAprilTagSimulator INSTANCE = Robot.isSimulation() ? new PhotonVisionAprilTagSimulator() : null;

    private final VisionSystemSim visionSim = new VisionSystemSim("simVision");
    private final Map<VisionCameraID, PhotonCameraSim> aprilTagCamSims = new HashMap<>();

    private static final Consumer<SimCameraProperties> defaultCameraSettings = cameraProp -> {
        // A 640 x 480 camera with a 100-degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
    };

    public static Optional<PhotonVisionAprilTagSimulator> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    private PhotonVisionAprilTagSimulator() {
        visionSim.addAprilTags(VisionUtil.APRIL_TAG_FIELD_LAYOUT);
    }

    public void addCamera(
            VisionCameraID cameraID,
            PhotonCamera camera,
            Transform3d transform,
            Consumer<SimCameraProperties> cameraSettingsOverride
            ) {
        SimCameraProperties cameraProp = new SimCameraProperties();

        if (cameraSettingsOverride == null) {
            cameraSettingsOverride = defaultCameraSettings;
        }

        cameraSettingsOverride.accept(cameraProp);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

        visionSim.addCamera(cameraSim, transform);
        aprilTagCamSims.put(cameraID, cameraSim);
    }

    public Optional<PhotonCameraSim> getSimCamera(VisionCameraID cameraID) {
        return Optional.ofNullable(aprilTagCamSims.get(cameraID));
    }

    public void update(Pose2d pose) {
        visionSim.update(pose);
    }
}
