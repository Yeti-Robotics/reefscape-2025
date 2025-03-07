package frc.robot.util.device.impl;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.device.DeviceBuilder;

public class TalonFXMotor extends DeviceBuilder<TalonFX, TalonFXConfiguration, TalonFXMotor> {
    private TalonFXMotor(TalonFX motor) {
        super(motor);
    }

    protected TalonFXConfiguration getDefaultConfig() {
        return new TalonFXConfiguration();
    }

    public static TalonFXMotor configure(int deviceID, String canbus) {
        return new TalonFXMotor(new TalonFX(deviceID, canbus));
    }

    public TalonFXMotor withCANCoder(CANcoder canCoder) {
        getConfig().Feedback.withFusedCANcoder(canCoder);
        return this;
    }

    @Override
    public TalonFXMotor syncConfigs() {
        getDevice().getConfigurator().apply(getConfig());
        return this;
    }

    public TalonFXMotor follow(TalonFX master) {
        return followWithRequest(new Follower(master.getDeviceID(), false));
    }

    public TalonFXMotor oppose(TalonFX master) {
        return followWithRequest(new Follower(master.getDeviceID(), true));
    }

    public TalonFXMotor followWithRequest(ControlRequest req) {
        getDevice().setControl(req);
        return this;
    }

    @Override
    protected TalonFXMotor getDeviceBuilderClass() {
        return this;
    }
}
