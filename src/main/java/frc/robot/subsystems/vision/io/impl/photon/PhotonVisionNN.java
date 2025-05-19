package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.data.VisionNNDetection;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonVisionNN extends AbstractPhotonProcessor implements VisionNNProcessor {
    private final PhotonCamera camera;
    private final List<VisionNNDetection> nnDetections = new ArrayList<>();
    private final String[] classNames;

    public PhotonVisionNN(PhotonCamera camera, String[] classNames) {
        super(camera);
        this.camera = camera;
        this.classNames = classNames;
    }

    @Override
    protected void runPreprocessing() {
        nnDetections.clear();
    }

    @Override
    protected void processResult(PhotonPipelineResult pipelineResult) {
        if (!pipelineResult.hasTargets()) return;

        for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
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
                    pipelineResult.getTimestampSeconds()));
        }
    }

    @Override
    public List<VisionNNDetection> getLatestNNDetections() {
        return nnDetections;
    }
}
