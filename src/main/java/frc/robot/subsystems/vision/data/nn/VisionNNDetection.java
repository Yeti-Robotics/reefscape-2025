package frc.robot.subsystems.vision.data.nn;

import frc.robot.subsystems.vision.data.VisionTimestampedResult;

public record VisionNNDetection(
        VisionNNCorner[] corners, double confidence, int classID, double area, double timestamp)
        implements VisionTimestampedResult {
    public record VisionNNCorner(double x, double y) {
    }

    private static double areaFromCorners(VisionNNCorner[] corners) {
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY,
                minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;

        for (VisionNNCorner corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.max(maxY, corner.y);
        }

        return (maxX - minX) * (maxY - minY);
    }

    public VisionNNDetection(
            VisionNNCorner[] corners, double confidence, int classID, double timestamp) {
        this(corners, confidence, classID, areaFromCorners(corners), timestamp);
    }
}
