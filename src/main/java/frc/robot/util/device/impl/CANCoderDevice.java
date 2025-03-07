package frc.robot.util.device.impl;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.util.device.DeviceBuilder;

public class CANCoderDevice extends DeviceBuilder<CANcoder, CANcoderConfiguration, CANCoderDevice> {
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
