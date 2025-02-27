package frc.robot.util.sim.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class AprilTagCamSimBuilder {
    private String cameraName = "";
    private Translation3d translation;
    private Rotation3d rotation;
    private Transform3d transform;

    private AprilTagCamSimBuilder() {}

    public static AprilTagCamSimBuilder newCamera() {
        return new AprilTagCamSimBuilder();
    }

    public AprilTagCamSimBuilder withTranslation(double x, double y, double z) {
        translation = new Translation3d(x, y, z);
        return this;
    }

    public AprilTagCamSimBuilder withRotation(double x, double y, double z) {
        rotation = new Rotation3d(x, y, z);
        return this;
    }

    public AprilTagCamSimBuilder withTransform(Transform3d t) {
        transform = t;
        return this;
    }

    public AprilTagCamSimBuilder withCameraName(String cameraName) {
        this.cameraName = cameraName;
        return this;
    }

    public AprilTagCamSim build() {
        if (translation == null) {
            translation = new Translation3d(0.0, 0.0, 0.0);
        }
        if (rotation == null) {
            rotation = new Rotation3d(0.0, 0.0, 0.0);
        }
        if (cameraName.isEmpty()) {
            cameraName = "camera";
        }

        Transform3d transform =
                this.transform == null ? new Transform3d(translation, rotation) : this.transform;

        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCamera cam = new PhotonCamera(cameraName);
        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, cameraProp);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        StructArrayPublisher<Pose3d> pub =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic(cameraName + "SeenTags", Pose3d.struct)
                        .publish();

        return new AprilTagCamSim(cam, cameraSim, pub, transform);
    }
}
