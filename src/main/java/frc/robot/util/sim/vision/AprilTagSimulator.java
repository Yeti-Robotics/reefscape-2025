package frc.robot.util.sim.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.subsystems.vision.apriltag.impl.photon.PhotonAprilTagSystem;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagSimulator {
    private final PhotonAprilTagSystem radioCam;
    private final PhotonAprilTagSystem scoreCam;
    VisionSystemSim visionSim;
    List<AprilTagCamSim> aprilTagCamSims;
    private final StructArrayPublisher<Pose3d> tagPublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("TagPoses", Pose3d.struct)
                    .publish();

    public AprilTagSimulator(PhotonAprilTagSystem radioCam, PhotonAprilTagSystem scoreCam) {
        visionSim = new VisionSystemSim("main");
        aprilTagCamSims = new ArrayList<>();
        this.radioCam = radioCam;
        this.scoreCam = scoreCam;
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    public void addCamera(AprilTagCamSim aprilTagCamSim) {
        visionSim.addCamera(aprilTagCamSim.getCameraSim(), aprilTagCamSim.getTransform());
        aprilTagCamSims.add(aprilTagCamSim);
    }

    /**
     * Update the simulator with the current pose of the robot.
     *
     * @param pose The current pose of the robot.
     */
    public void update(Pose2d pose) {
        visionSim.update(pose);
    }

    public List<AprilTagCamSim> getAprilTagCamSims() {
        return aprilTagCamSims;
    }
}
