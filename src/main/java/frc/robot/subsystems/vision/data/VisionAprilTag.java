package frc.robot.subsystems.vision.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public record VisionAprilTag(int fiducialID,  Pose2d robotToTargetPose, double ambiguity) {
    public Distance getDistanceMeasure() {
        return Units.Meters.of(robotToTargetPose.getTranslation().getDistance(Translation2d.kZero));
    }
}
