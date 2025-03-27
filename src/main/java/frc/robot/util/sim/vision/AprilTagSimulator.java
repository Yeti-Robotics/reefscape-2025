package frc.robot.util.sim.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.List;

public class AprilTagSimulator extends SubsystemBase {
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

    public void update(Pose2d pose) {
        visionSim.update(pose);
    }

    public List<AprilTagCamSim> getAprilTagCamSims() {
        return aprilTagCamSims;
    }
}
