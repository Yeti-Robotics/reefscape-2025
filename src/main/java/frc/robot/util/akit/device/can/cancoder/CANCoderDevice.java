package frc.robot.util.akit.device.can.cancoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANDeviceBuilder;

public class CANCoderDevice
        extends CANDeviceBuilder<
                CANcoder, CANcoderConfiguration, CANCoderDeviceInputs, CANCoderDevice> {
    private CANCoderDevice(CANcoder device) {
        super(device);
    }

    public static CANCoderDevice configure(int deviceID, String canBus) {
        return new CANCoderDevice(new CANcoder(deviceID, canBus));
    }

    @Override
    public CANCoderDevice syncConfigs() {
        getDevice().getConfigurator().apply(getConfig());
        return this;
    }

    @Override
    protected CANCoderDeviceInputs createDeviceInputs() {
        return new CANCoderDeviceInputsAutoLogged();
    }

    @Override
    protected DeviceLogger<CANCoderDeviceInputs> getLogger() {
        return new CANCoderDeviceLogger(getDevice());
    }

    protected CANcoderConfiguration getDefaultConfig() {
        return new CANcoderConfiguration()
                .withMagnetSensor(
                        new MagnetSensorConfigs()
                                .withSensorDirection(
                                        SensorDirectionValue.CounterClockwise_Positive));
    }

    @Override
    protected CANCoderDevice getDeviceBuilderClass() {
        return this;
    }
}
