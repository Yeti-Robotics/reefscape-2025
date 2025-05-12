package frc.robot.subsystems.vision.data;

public record VisionNNDetection(
        VisionNNCorner[] corners, double confidence, int classID, String className, double area, double timestamp)
        implements VisionTimestampedResult {
    public record VisionNNCorner(double x, double y) {}

    private static double areaFromCorners(VisionNNCorner[] corners) {
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;

        for (VisionNNCorner corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.max(maxY, corner.y);
        }

        return (maxX - minX) * (maxY - minY);
    }

    private static String getClassName(int classID, String[] classNames) {
        return classID > 0 && classID < classNames.length ? classNames[classID] : "Unknown";
    }

    public VisionNNDetection(
            VisionNNCorner[] corners,
            double confidence,
            int classID,
            String[] classNames,
            double area,
            double timestamp) {
        this(corners, confidence, classID, getClassName(classID, classNames), area, timestamp);
    }

    public VisionNNDetection(
            VisionNNCorner[] corners, double confidence, int classID, String className, double timestamp) {
        this(corners, confidence, classID, className, areaFromCorners(corners), timestamp);
    }
}
