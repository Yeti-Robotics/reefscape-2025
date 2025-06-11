package frc.robot.subsystems.vision.io.impl.photon;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

public abstract class AbstractPhotonProcessor implements VisionProcessor {
    protected final PhotonCamera photonCamera;
    private double latencyMs = 0;

    public AbstractPhotonProcessor(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
    }

    protected abstract void processResult(PhotonPipelineResult pipelineResult);
    protected void runPreprocessing() {}
    protected void runPostProcessing() {}

    @Override
    final public void visionPeriodic() {
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        double totalLatency = 0;
        runPreprocessing();

        for (PhotonPipelineResult result : results) {
            processResult(result);
            totalLatency += result.metadata.getLatencyMillis();
        }

        if (!results.isEmpty()) {
            latencyMs = totalLatency / results.size();
        }

        runPostProcessing();
    }

    @Override
    public double latencyMs() {
        return latencyMs;
    }
}
