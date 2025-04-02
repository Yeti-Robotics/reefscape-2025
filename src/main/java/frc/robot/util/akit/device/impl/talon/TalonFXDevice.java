package frc.robot.util.akit.device.impl.talon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.robot.util.akit.device.impl.DeviceBuilder;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputsAutoLogged;
import frc.robot.util.akit.device.impl.DeviceLogger;

public class TalonFXDevice
        extends DeviceBuilder<TalonFX, TalonFXConfiguration, TalonFXDeviceInputs, TalonFXDevice> {
    private TalonFXDevice(TalonFX motor) {
        super(motor);
    }

    protected TalonFXConfiguration getDefaultConfig() {
        return new TalonFXConfiguration();
    }

    public static TalonFXDevice configure(int deviceID, String canBus) {
        return new TalonFXDevice(new TalonFX(deviceID, canBus));
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
    protected DeviceLogger<TalonFXDeviceInputs> createLogger() {
        return new TalonFXDeviceLogger(getDevice());
    }

    public TalonFXDevice follow(TalonFX master) {
        return followWithRequest(master.getDeviceID(), false);
    }

    public TalonFXDevice oppose(TalonFX master) {
        return followWithRequest(master.getDeviceID(), true);
    }

    public TalonFXDevice usingFusedCANcoder(CANcoder cancoder) {
        return usingCANcoder(cancoder, FeedbackSensorSourceValue.FusedCANcoder);
    }

    public TalonFXDevice usingFusedCANcoder(int cancoderID) {
        return usingCANcoder(cancoderID, FeedbackSensorSourceValue.FusedCANcoder);
    }

    public TalonFXDevice usingCANcoder(CANcoder cancoder, FeedbackSensorSourceValue source) {
        return usingCANcoder(cancoder.getDeviceID(), source);
    }

    public TalonFXDevice usingCANcoder(int cancoderID, FeedbackSensorSourceValue source) {
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
