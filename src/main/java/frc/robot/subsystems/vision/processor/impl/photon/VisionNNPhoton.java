package frc.robot.subsystems.vision.processor.impl.photon;

import frc.robot.subsystems.vision.data.VisionNNDetection;
import frc.robot.subsystems.vision.processor.VisionNNProcessor;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class VisionNNPhoton implements VisionNNProcessor {
    private final PhotonCamera camera;
    private final List<VisionNNDetection> nnDetections = new ArrayList<>();
    private final String[] classNames;

    public VisionNNPhoton(PhotonCamera camera, String[] classNames) {
        this.camera = camera;
        this.classNames = classNames;
    }

    @Override
    public void visionPeriodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) return;

        nnDetections.clear();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            for (PhotonTrackedTarget target : result.getTargets()) {
                int numCorners = target.minAreaRectCorners.size(); // should be four but you never know
                VisionNNDetection.VisionNNCorner[] corners = new VisionNNDetection.VisionNNCorner[numCorners];

                for (int i = 0; i < numCorners; i++) {
                    corners[i] = new VisionNNDetection.VisionNNCorner(
                            target.minAreaRectCorners.get(i).x, target.minAreaRectCorners.get(i).y);
                }

                nnDetections.add(new VisionNNDetection(
                        corners,
                        target.objDetectConf,
                        target.objDetectId,
                        classNames,
                        target.area,
                        result.getTimestampSeconds()));
            }
        }
    }

    @Override
    public List<VisionNNDetection> getLatestNNDetections() {
        return nnDetections;
    }
}
