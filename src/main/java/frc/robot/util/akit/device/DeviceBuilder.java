package frc.robot.util.akit.device;

public abstract class DeviceBuilder<D, I extends DeviceInputs, U extends DeviceBuilder<D, I, U>> {
    protected final D device;
    protected final I inputs;
    private  boolean loggingAdded = false;

    protected DeviceBuilder(D device) {
        this.device = device;
        this.inputs = createDeviceInputs();
    }

    protected abstract U getDeviceBuilderClass();

    public U log(String key) {
        if (!loggingAdded) {
            DeviceLogging.addLoggerWithInputs(key, createLogger(), getDeviceInputs());
            loggingAdded = true;
        }

        return getDeviceBuilderClass();
    }

    public I getDeviceInputs() {
        return inputs;
    }

    public D getDevice() {
        return device;
    }

    protected abstract I createDeviceInputs();

    protected abstract DeviceLogger<I> createLogger();
}
