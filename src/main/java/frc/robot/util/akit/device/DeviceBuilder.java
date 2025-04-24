package frc.robot.util.akit.device;

public abstract class DeviceBuilder<D, I extends DeviceInputs, U extends DeviceBuilder<D, I, U>> {
    private final D device;

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
        DeviceLogging.addLoggerWithInputs(key, getLogger(), createDeviceInputs());
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
}
