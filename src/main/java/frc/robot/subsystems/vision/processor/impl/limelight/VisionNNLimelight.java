package frc.robot.subsystems.vision.processor.impl.limelight;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.data.VisionNNDetection;
import frc.robot.subsystems.vision.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.processor.impl.limelight.util.LimelightHelpers;

import java.util.Arrays;
import java.util.List;

public class VisionNNLimelight implements VisionNNProcessor {
    private final String limelightName;
    private final String[] classNames;

    public VisionNNLimelight(String limelightName, String[] classNames) {
        this.limelightName = limelightName;
        this.classNames = classNames;
    }

    @Override
    public List<VisionNNDetection> getLatestNNDetections() {
        LimelightHelpers.RawDetection[] rawDetections = LimelightHelpers.getRawDetections(limelightName);

        // TODO: figure out how to get confidence from raw detections, or use JSON
        return Arrays.stream(rawDetections)
                .map(d -> new VisionNNDetection(
                        new VisionNNDetection.VisionNNCorner[] {
                            new VisionNNDetection.VisionNNCorner(d.corner0_X, d.corner0_Y),
                            new VisionNNDetection.VisionNNCorner(d.corner1_X, d.corner1_Y),
                            new VisionNNDetection.VisionNNCorner(d.corner2_X, d.corner2_Y),
                            new VisionNNDetection.VisionNNCorner(d.corner3_X, d.corner3_Y)
                        },
                        0, // no confidence from raw detections! what is Limelight doing?!
                        d.classId,
                        classNames,
                        d.ta,
                        Timer.getFPGATimestamp()))
                .toList();
    }
}
