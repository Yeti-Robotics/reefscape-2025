package frc.robot.util.akit.device.impl;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputs;
import frc.robot.util.akit.device.inputs.TalonFXDeviceInputsAutoLogged;
import frc.robot.util.akit.device.log.DeviceLogger;
import frc.robot.util.akit.device.log.DeviceLoggingRegistry;
import frc.robot.util.akit.device.log.TalonFXDeviceLogger;

public class TalonFXMotor extends DeviceBuilder<TalonFX, TalonFXConfiguration, TalonFXDeviceInputs, TalonFXMotor> {
    private TalonFXMotor(TalonFX motor) {
        super(motor);
    }

    protected TalonFXConfiguration getDefaultConfig() {
        return new TalonFXConfiguration();
    }

    public static TalonFXMotor configure(int deviceID, String canBus) {
        return new TalonFXMotor(new TalonFX(deviceID, canBus));
    }

    @Override
    public TalonFXMotor syncConfigs() {
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

    public TalonFXMotor follow(TalonFX master) {
        return followWithRequest(master.getDeviceID(), false);
    }

    public TalonFXMotor oppose(TalonFX master) {
        return followWithRequest(master.getDeviceID(), true);
    }

    public TalonFXMotor usingFusedCANcoder(CANcoder cancoder) {
        return usingCANcoder(cancoder, FeedbackSensorSourceValue.FusedCANcoder);
    }

    public TalonFXMotor usingFusedCANcoder(int cancoderID) {
        return usingCANcoder(cancoderID, FeedbackSensorSourceValue.FusedCANcoder);
    }

    public TalonFXMotor usingCANcoder(CANcoder cancoder, FeedbackSensorSourceValue source) {
        return usingCANcoder(cancoder.getDeviceID(), source);
    }

    public TalonFXMotor usingCANcoder(int cancoderID, FeedbackSensorSourceValue source) {
        getConfig().Feedback.FeedbackSensorSource = source;
        getConfig().Feedback.FeedbackRemoteSensorID = cancoderID;
        return this;
    }

    private TalonFXMotor followWithRequest(int primaryDeviceID, boolean oppose) {
        getDevice().setControl(new Follower(primaryDeviceID, oppose));
        return this;
    }

    @Override
    protected TalonFXMotor getDeviceBuilderClass() {
        return this;
    }
}
