package frc.robot.util.sim.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagCamSim {
    private final PhotonCamera cam;
    private final PhotonCameraSim cameraSim;
    private final List<Integer> tagCache;
    private final StructArrayPublisher<Pose3d> pub;
    private final Transform3d transform;

    public AprilTagCamSim(
            PhotonCamera cam,
            PhotonCameraSim cameraSim,
            StructArrayPublisher<Pose3d> pub,
            Transform3d transform) {
        this.cam = cam;
        this.cameraSim = cameraSim;
        this.tagCache = new ArrayList<>();
        this.pub = pub;
        this.transform = transform;
    }

    public void publishSeenTags() {
        var results = cam.getAllUnreadResults();
        if (!results.isEmpty()) {
            tagCache.clear();
            var t =
                    results.get(0).getTargets().stream()
                            .map(PhotonTrackedTarget::getFiducialId)
                            .toList();
            tagCache.addAll(t);
        }

        List<Pose3d> seenTags = new ArrayList<>();
        var tags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTags();
        for (int i = 0; i < tagCache.size(); i++) {
            var seenTagIdsArr = tagCache.toArray(Integer[]::new);
            seenTags.add(tags.get(seenTagIdsArr[i] - 1).pose);
        }

        pub.set(seenTags.toArray(Pose3d[]::new));
    }

    public static List<Pose3d> publishSeenTags(AprilTagResults results) {
        List<Pose3d> seenTags = new ArrayList<>();
        var tags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTags();
        for (int i = 0; i < results.results().size(); i++) {
            var seenTagIdsArr =
                    results.results().stream()
                            .map(AprilTagDetection::getFiducialID)
                            .toArray(Integer[]::new);
            seenTags.add(tags.get(seenTagIdsArr[i] - 1).pose);
        }

        return seenTags;
    }

    public Transform3d getTransform() {
        return transform;
    }

    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    public PhotonCamera getCam() {
        return cam;
    }
}
