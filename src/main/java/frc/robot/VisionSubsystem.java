package frc.robot;

import edu.wpi.first.wpilibj2.command.*;

public class VisionSubsystem extends SubsystemBase {
    public class VisionConstants {
        public static final String PICK_LIMELIGHT = "pick limelight";
        public static final String PLACE_LIMELIGHT = "place limelight";
    }

    public double getPlaceX() {
        return LimelightHelpers.getTX(VisionConstants.PLACE_LIMELIGHT);
    }

    public double getPlaceY() {
        return LimelightHelpers.getTY(VisionConstants.PLACE_LIMELIGHT);
    }

    public LimelightHelpers.LimelightResults getTargetingResults() {
        return LimelightHelpers.getLatestResults(VisionConstants.PLACE_LIMELIGHT);
    }
}
