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

    public U syncConfigs() {
        return getDeviceBuilderClass();
    }

    public U log(String key) {
        DeviceLoggingRegistry.get().addLoggerWithInputs(key, createLogger(), createDeviceInputs());
        return getDeviceBuilderClass();
    }

    protected abstract I createDeviceInputs();

    protected abstract DeviceLogger<I> createLogger();

    protected abstract C getDefaultConfig();

    public D getDevice() {
        return device;
    }

    protected C getConfig() {
        if (config == null) {
            config = getDefaultConfig();
        }

        return config;
    }

    public U extend(Consumer<C> configConsumer) {
        configConsumer.accept(getConfig());
        return getDeviceBuilderClass();
    }

    public U using(C config) {
        this.config = config;
        return getDeviceBuilderClass();
    }
}
