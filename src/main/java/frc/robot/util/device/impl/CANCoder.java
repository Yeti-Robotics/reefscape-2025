package frc.robot.util.device.impl;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.util.device.DeviceBuilder;

public class CANCoder extends DeviceBuilder<CANcoder, CANcoderConfiguration, CANCoder> {
    private CANCoder(CANcoder device) {
        super(device);
    }

    public static CANCoder configure(int deviceID, String canBus) {
        return new CANCoder(new CANcoder(deviceID, canBus));
    }

    @Override
    public CANCoder syncConfigs() {
        getDevice().getConfigurator().apply(getConfig());
        return this;
    }

    protected CANcoderConfiguration getDefaultConfig() {
        return new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    @Override
    protected CANCoder getDeviceBuilderClass() {
        return this;
    }
}
