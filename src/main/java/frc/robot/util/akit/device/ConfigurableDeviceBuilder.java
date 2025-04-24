package frc.robot.util.akit.device;

import java.util.function.Consumer;

public abstract class ConfigurableDeviceBuilder<
                D, C, I extends DeviceInputs, U extends DeviceBuilder<D, I, U>>
        extends DeviceBuilder<D, I, U> {
    protected C config;

    protected ConfigurableDeviceBuilder(D device) {
        super(device);
    }

    protected abstract C getDefaultConfig();

    protected C getConfig() {
        if (config == null) {
            config = getDefaultConfig();
        }

        return config;
    }

    /**
     * Modify the currently used configuration, useful when reusing the same configuration object
     * for multiple devices
     *
     * @param configConsumer - function to modify the currently used configuration
     * @return the device builder
     */
    public U extendConfig(Consumer<C> configConsumer) {
        configConsumer.accept(getConfig());
        return getDeviceBuilderClass();
    }

    /**
     * Provide the device with a configuration to use.
     *
     * <p>Note: if you want to reuse a configuration object for multiple CAN devices, make sure to
     * call {@link frc.robot.util.akit.device.can.CANDeviceBuilder#syncConfigs()} first.
     *
     * @param config Configuration object you want to use
     * @return the device builder
     */
    public U withConfig(C config) {
        this.config = config;
        return getDeviceBuilderClass();
    }
}
