package frc.robot.util.sim.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagSimulator {
    VisionSystemSim visionSim;
    List<AprilTagCamSim> aprilTagCamSims;

    public AprilTagSimulator() {
        visionSim = new VisionSystemSim("main");
        aprilTagCamSims = new ArrayList<>();

        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    public void addCamera(AprilTagCamSim aprilTagCamSim) {
        visionSim.addCamera(aprilTagCamSim.getCameraSim(), aprilTagCamSim.getTransform());
        aprilTagCamSims.add(aprilTagCamSim);
    }

    /* <<<<<<<<<<<<<<  ✨ Windsurf Command ⭐ >>>>>>>>>>>>>>>> */
    /**
     * Update the simulator with the current pose of the robot.
     *
     * @param pose The current pose of the robot.
     */
    /* <<<<<<<<<<  07c48757-5cd3-4ab9-9339-a50b85422bce  >>>>>>>>>>> */
    public void update(Pose2d pose) {
        visionSim.update(pose);
        aprilTagCamSims.forEach(AprilTagCamSim::publishSeenTags);
    }

    public List<AprilTagCamSim> getAprilTagCamSims() {
        return aprilTagCamSims;
    }
}
