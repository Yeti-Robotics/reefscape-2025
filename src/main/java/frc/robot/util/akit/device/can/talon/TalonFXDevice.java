package frc.robot.util.akit.device.can.talon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Frequency;
import frc.robot.Robot;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANDeviceBuilder;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.TalonFXSimProfile;
import java.util.function.Function;

public class TalonFXDevice
        extends CANDeviceBuilder<
                TalonFX, TalonFXConfiguration, TalonFXDeviceInputs, TalonFXDevice> {
    private TalonFXSimProfile simProfile;

    private TalonFXDevice(TalonFX motor) {
        super(motor);

        if (Robot.isSimulation()) {
            simProfile = PhysicsSim.getInstance().addTalonFX(motor);
        }
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

    public TalonFXDevice withCANCoder(CANcoder cancoder, FeedbackSensorSourceValue source) {
        TalonFXConfiguration config = getConfig();
        config.Feedback.FeedbackSensorSource = source;
        config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();

        if (simProfile != null) {
            simProfile.setCancoder(cancoder);
        }

        return this;
    }

    public TalonFXDevice optimizeBusUtilization() {
        // ensure important status signals are enabled before optimizing
        BaseStatusSignal.setUpdateFrequencyForAll(
                TalonFXDeviceLogger.DEFAULT_UPDATE_HZ,
                getDevice().getDutyCycle(),
                getDevice().getTorqueCurrent(),
                getDevice().getMotorVoltage(),
                getDevice().getPosition(),
                getDevice().getVelocity(),
                getDevice().getAcceleration());

        getDevice().optimizeBusUtilization();
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
