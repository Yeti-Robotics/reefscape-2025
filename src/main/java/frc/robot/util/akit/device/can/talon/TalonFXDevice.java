package frc.robot.util.akit.device.can.talon;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Frequency;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANDeviceBuilder;
import java.util.function.Function;
import static frc.robot.util.akit.device.can.CANDeviceBuilder.CONNECTED_DEBOUNCE_TIME;

public class TalonFXDevice
        extends CANDeviceBuilder<
                TalonFX, TalonFXConfiguration, TalonFXDeviceInputs, TalonFXDevice> {
    private TalonFXDevice(TalonFX motor) {
        super(motor);
    }

    protected TalonFXConfiguration getDefaultConfig() {
        return new TalonFXConfiguration();
    }

    public static TalonFXDevice configure(int deviceID, String canBus) {
        return new TalonFXDevice(new TalonFX(deviceID, canBus));
    }

    public static TalonFXDevice from(TalonFX motor) {
        return new TalonFXDevice(motor);
    }

    @Override
    public TalonFXDevice syncConfigs() {
        getDevice().getConfigurator().apply(getConfig());
        return this;
    }

    @Override
    protected TalonFXDeviceInputs createDeviceInputs() {
        return new TalonFXDeviceInputsAutoLogged();
    }

    @Override
    protected DeviceLogger<TalonFXDeviceInputs> getLogger() {
        return new TalonFXDeviceLogger(getDevice());
    }

    public TalonFXDevice follow(TalonFX master) {
        return followWithRequest(master.getDeviceID(), false);
    }

    public TalonFXDevice oppose(TalonFX master) {
        return followWithRequest(master.getDeviceID(), true);
    }

    public TalonFXDevice withStatusSignalFrequency(
            Frequency frequency, Function<TalonFX, StatusSignal<?>> statusSignalFunction) {
        statusSignalFunction.apply(getDevice()).setUpdateFrequency(frequency);
        return this;
    }

    public TalonFXDevice withFusedCANcoder(CANcoder cancoder) {
        return withCANCoder(cancoder, FeedbackSensorSourceValue.FusedCANcoder);
    }

    public TalonFXDevice withFusedCANcoder(int cancoderID) {
        return withCANCoder(cancoderID, FeedbackSensorSourceValue.FusedCANcoder);
    }

    public TalonFXDevice withCANCoder(CANcoder cancoder, FeedbackSensorSourceValue source) {
        return withCANCoder(cancoder.getDeviceID(), source);
    }

    public TalonFXDevice withCANCoder(int cancoderID, FeedbackSensorSourceValue source) {
        TalonFXConfiguration config = getConfig();
        config.Feedback.FeedbackSensorSource = source;
        config.Feedback.FeedbackRemoteSensorID = cancoderID;
        return this;
    }

    private TalonFXDevice followWithRequest(int primaryDeviceID, boolean oppose) {
        getDevice().setControl(new Follower(primaryDeviceID, oppose));
        return this;
    }

    @Override
    protected TalonFXDevice getDeviceBuilderClass() {
        return this;
    }
}
