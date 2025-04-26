package frc.robot.util.akit.device.can;

import com.ctre.phoenix6.StatusCode;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CANUtil {
    public static final double CONNECTED_DEBOUNCE_TIME = 0.5;
    public static final double CANCODER_DEFAULT_UPDATE_HZ = 250.0;
    public static final double TALON_DEFAULT_UPDATE_HZ = 250.0;
    public static final int DEFAULT_RETRY_ATTEMPTS = 5;

    public static void tryUntilOk(int attempts, BooleanSupplier isOK) {
        for (int i = 0; i < attempts; i++) {
            if (isOK.getAsBoolean()) return;
        }
    }

    public static void tryUntilOk(int attempts, Supplier<StatusCode> cmd) {
        tryUntilOk(attempts, () -> cmd.get().isOK());
    }

    public static void tryUntilOk(Supplier<StatusCode> cmd) {
        tryUntilOk(DEFAULT_RETRY_ATTEMPTS, cmd);
    }

    public static void tryUntilOk(BooleanSupplier isOK) {
        tryUntilOk(DEFAULT_RETRY_ATTEMPTS, isOK);
    }
}
