package frc.robot.util.akit.device;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.akit.device.can.cancoder.CANCoderDevice;
import frc.robot.util.akit.device.can.cancolor.CANColorDevice;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;
import frc.robot.util.akit.device.digital.DigitalInputDevice;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class DeviceLogging {
    private static final ArrayList<LoggingEntry<?>> loggers = new ArrayList<>();
    private static boolean disable = false;

    private record LoggingEntry<T extends DeviceInputs>(String key, DeviceLogger<T> logger, T inputs) {
        public T getUpdatedInputs() {
            logger.updateInputs(inputs);
            return inputs;
        }
    }

    protected static <T extends DeviceInputs> void addLoggerWithInputs(String key, DeviceLogger<T> logger, T inputs) {
        if (disable) return;

        loggers.add(new LoggingEntry<>(key, logger, inputs));
    }

    /**
     * @apiNote call this method as frequently as you would like to log device data
     */
    public static void updateDeviceLogging() {
        if (disable) return;

        for (LoggingEntry<?> entry : loggers) {
            Logger.processInputs(entry.key, entry.getUpdatedInputs());
        }
    }

    /**
     * @apiNote disables logging for ALL devices, use only if you don't want to log anything
     */
    public static void disableDeviceLogging() {
        disable = true;
        loggers.clear();
    }

    public void log(String key, DigitalInput digitalInput) {
        DigitalInputDevice.from(digitalInput).log(key);
    }

    public void log(String key, TalonFX talonFX) {
        TalonFXDevice.from(talonFX).log(key);
    }

    public void log(String key, Canandcolor canandcolor) {
        CANColorDevice.from(canandcolor).log(key);
    }

    public void log(String key, CANcoder cancoder) {
        CANCoderDevice.from(cancoder).log(key);
    }
}
