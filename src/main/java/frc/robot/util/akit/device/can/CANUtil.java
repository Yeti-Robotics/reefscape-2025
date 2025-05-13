package frc.robot.util.akit.device.can;

import java.util.function.BooleanSupplier;

public class CANUtil {
    public static final double CONNECTED_DEBOUNCE_TIME = 0.5;
    public static final double CANCODER_DEFAULT_UPDATE_HZ = 250.0;
    public static final double TALON_DEFAULT_UPDATE_HZ = 50.0;
    public static final int DEFAULT_RETRY_ATTEMPTS = 5;

    public static void tryUntilOk(int attempts, BooleanSupplier isOK) {
        for (int i = 0; i < attempts; i++) {
            if (isOK.getAsBoolean()) return;
        }
    }

    public static void tryUntilOk(BooleanSupplier isOK) {
        tryUntilOk(DEFAULT_RETRY_ATTEMPTS, isOK);
    }
}
