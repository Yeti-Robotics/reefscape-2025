package frc.robot.util.akit.device.can.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANUtil;
import frc.robot.util.akit.device.can.CANDeviceBuilder;

public class CANCoderDevice
        extends CANDeviceBuilder<CANcoder, CANcoderConfiguration, CANCoderDeviceInputs, CANCoderDevice> {
    private CANCoderDevice(CANcoder device) {
        super(device);
    }

    public static CANCoderDevice configure(int deviceID, String canBus) {
        return from(new CANcoder(deviceID, canBus));
    }

    public static CANCoderDevice from(CANcoder cancoder) {
        return new CANCoderDevice(cancoder);
    }

    /**
     * @apiNote <p>Make sure this is always the last call you make before {@link
     * CANCoderDevice#getDevice}, otherwise logging and other status signals won't work
     */
    public CANCoderDevice optimizeBusUtilization() {
        // enable important signals before optimizing
        BaseStatusSignal.setUpdateFrequencyForAll(
                CANUtil.CANCODER_DEFAULT_UPDATE_HZ,
                getDevice().getAbsolutePosition(),
                getDevice().getPosition());
        getDevice().optimizeBusUtilization();
        return this;
    }

    @Override
    public CANCoderDevice syncConfigs() {
        CANUtil.tryUntilOk(() -> getDevice().getConfigurator().apply(getConfig()));
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
                        new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    @Override
    protected CANCoderDevice getDeviceBuilderClass() {
        return this;
    }
}
