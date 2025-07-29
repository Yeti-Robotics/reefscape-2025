package frc.robot.util.sim.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
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

    /**
     * Update the simulator with the current pose of the robot.
     *
     * @param pose The current pose of the robot.
     */
    public void update(Pose2d pose) {
        visionSim.update(pose);
        
        for (AprilTagCamSim camSim : aprilTagCamSims) {
            var results = camSim.getCam().getAllUnreadResults();
            if (!results.isEmpty()) {
                List<AprilTagDetection> detections = new ArrayList<>();
                for (var target : results.get(0).getTargets()) {
                    Transform3d cameraToTarget = target.getBestCameraToTarget();
                    Pose2d targetPose = new Pose2d(
                        cameraToTarget.getX(),
                        cameraToTarget.getY(),
                        cameraToTarget.getRotation().toRotation2d()
                    );
                    detections.add(new AprilTagDetection(target.getFiducialId(), pose, targetPose, target.getPoseAmbiguity()));
                }
                
                AprilTagResults aprilTagResults = new AprilTagResults(Timer.getFPGATimestamp(), 20, detections);
                List<Pose3d> seenTags = AprilTagCamSim.publishSeenTags(aprilTagResults);
                
                seenTags.forEach(camSim::publishSeenTags);
            }
        }
    }

    public List<AprilTagCamSim> getAprilTagCamSims() {
        return aprilTagCamSims;
    }
}
