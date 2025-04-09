package frc.robot.util.akit.device;

import java.util.function.Consumer;

public abstract class DeviceBuilder<
        D, C, I extends DeviceInputs, U extends DeviceBuilder<D, C, I, U>> {
    private final D device;
    private C config;

    protected DeviceBuilder(D device) {
        this.device = device;
    }

    protected abstract U getDeviceBuilderClass();

    /**
     * Logs this device with its corresponding {@code DeviceInputs} class
     *
     * @param key Name of the key you want to log into AdvantageKit
     * @return the device builder
     */
    public U log(String key) {
        DeviceLoggingRegistry.get().addLoggerWithInputs(key, getLogger(), createDeviceInputs());
        return getDeviceBuilderClass();
    }

    protected abstract I createDeviceInputs();

    protected abstract DeviceLogger<I> getLogger();

    /**
     * Get your currently used device
     *
     * @return device
     */
    public D getDevice() {
        return device;
    }

    protected abstract C getDefaultConfig();

    protected C getConfig() {
        if (!hasConfig()) {
            config = getDefaultConfig();
        }

        return config;
    }

    protected boolean hasConfig() {
        return config != null;
    }

    /**
     * Modify the currently used configuration, useful when reusing the same configuration object
     * for multiple devices
     *
     * @param configConsumer - function to modify the currently used configuration
     * @return the device builder
     */
    public U extend(Consumer<C> configConsumer) {
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
    public U using(C config) {
        this.config = config;
        return getDeviceBuilderClass();
    }
}
