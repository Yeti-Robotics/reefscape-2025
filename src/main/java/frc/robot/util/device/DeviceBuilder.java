package frc.robot.util.device;

import java.util.function.Consumer;

public abstract class DeviceBuilder<D, C, U extends DeviceBuilder<D, C, U>> {
    private final D device;
    private C config;

    protected DeviceBuilder(D device) {
        this.device = device;
    }

    protected abstract U getDeviceBuilderClass();

    public abstract U syncConfigs();

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
