package frc.robot.constants;

import java.util.Map;

public final class OIConstants {

    public static final Map<Integer, ControllerType> CONTROLLERS =
            Map.of(0, ControllerType.CUSTOM, 1, ControllerType.XBOX);

    public static final int CONTROLLER_COUNT = CONTROLLERS.size(); // placehold
    public static final double DEADBAND = .05; // placehold
    public static final String Translational_XSupplier = "translationXSupplier";
    public static final String Translational_YSupplier = "translationYSupplier";
    public static final String THETA_SUPPLIER = "thetaSupplier";

    public enum ControllerType {
        CUSTOM,
        XBOX
    }

    public enum ButtonMode {
        PRIMARY
    }
}
