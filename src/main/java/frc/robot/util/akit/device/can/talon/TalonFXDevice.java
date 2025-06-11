package frc.robot.util.akit.device.can.talon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Robot;
import frc.robot.util.akit.device.DeviceLogger;
import frc.robot.util.akit.device.can.CANDeviceBuilder;
import frc.robot.util.akit.device.can.CANUtil;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.TalonFXSimProfile;
import org.littletonrobotics.junction.Logger;

import java.util.EnumSet;
import java.util.function.Function;
import java.util.function.Supplier;

public class TalonFXDevice extends CANDeviceBuilder<TalonFX, TalonFXConfiguration, TalonFXDeviceInputs, TalonFXDevice> {
    public enum TalonFXLogging {
        PID,
        MOTOR,
        POSITION
    }

    private static final EnumSet<TalonFXLogging> talonFXDefaultLogSettings = EnumSet.of(TalonFXLogging.POSITION, TalonFXLogging.MOTOR);
    private TalonFXLogging[] loggingSettings;
    private TalonFXSimProfile simProfile;

    private TalonFXDevice(TalonFX motor) {
        super(motor);

        if (Robot.isSimulation()) {
            simProfile = PhysicsSim.getInstance().addTalonFX(motor);
        }
    }

    public static void enablePIDDebugging() {
        talonFXDefaultLogSettings.add(TalonFXLogging.PID);
    }

    public static TalonFXDevice configure(int deviceID, String canBus) {
        return new TalonFXDevice(new TalonFX(deviceID, canBus));
    }

    public static TalonFXDevice from(TalonFX motor) {
        return new TalonFXDevice(motor);
    }

    protected TalonFXConfiguration getDefaultConfig() {
        return new TalonFXConfiguration();
    }

    @Override
    protected boolean doConfigSync() {
        return device.getConfigurator().apply(getConfig()).isOK();
    }

    public TalonFXDevice follow(TalonFX master) {
        return followWithRequest(master.getDeviceID(), false);
    }

    public TalonFXDevice oppose(TalonFX master) {
        return followWithRequest(master.getDeviceID(), true);
    }

    /**
     * @param frequency                     frequency to run status signal updates
     * @param statusSignalFunctionReference Method reference on TalonFX device to get corresponding status signal
     * @param <T>                           value type
     * @return supplier function with status signal value
     */
    public <T> Supplier<T> mapStatusSignalWithInputs(
            Frequency frequency, Function<TalonFX, StatusSignal<T>> statusSignalFunctionReference, Function<TalonFXDeviceInputs, T> inputsFunctionReference) {
        return mapStatusSignalWithInputs(frequency.in(Units.Hertz), statusSignalFunctionReference, inputsFunctionReference);
    }

    /**
     * @param frequency                     frequency to run status signal updates
     * @param statusSignalFunctionReference Method reference on TalonFX device to get corresponding status signal
     * @param <T>                           value type
     * @return supplier function with status signal value
     */
    public <T> Supplier<T> mapStatusSignalWithInputs(
            double frequency, Function<TalonFX, StatusSignal<T>> statusSignalFunctionReference, Function<TalonFXDeviceInputs, T> inputsFunctionReference) {
        StatusSignal<T> statusSignal = statusSignalFunctionReference.apply(device);
        statusSignal.setUpdateFrequency(frequency);

        return Logger.hasReplaySource() ? statusSignal::getValue : () -> inputsFunctionReference.apply(inputs);
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

    /**
     * @param key   Logging key used to identify this device in AdvantageScope
     * @param modes Logging types you want to enable for this device, this will override default settings
     * @return builder for chaining
     */
    public TalonFXDevice logEnable(String key, TalonFXLogging... modes) {
        loggingSettings = modes;
        return super.log(key);
    }

    @Override
    protected TalonFXDeviceInputs createDeviceInputs() {
        return new TalonFXDeviceInputsAutoLogged();
    }

    @Override
    protected DeviceLogger<TalonFXDeviceInputs> createLogger() {
        EnumSet<TalonFXLogging> logSettingSet = talonFXDefaultLogSettings;

        if (loggingSettings != null && loggingSettings.length > 0) {
            logSettingSet = EnumSet.of(loggingSettings[0], loggingSettings);
        }

        return new TalonFXDeviceLogger(device, logSettingSet);
    }

    /**
     * @apiNote <p>Make sure this is always the last call you make before {@link
     * TalonFXDevice#device}, otherwise logging and other status signals may not work
     * to ensure that the status signals you want to use work, call {@link TalonFXDevice#mapStatusSignalWithInputs(double, Function, Function)}
     * before calling this function to set an update frequency for the status signal you want to use
     */
    public TalonFXDevice optimizeBusUtilization() {
        // ensure important status signals are enabled before optimizing
        BaseStatusSignal.setUpdateFrequencyForAll(
                CANUtil.TALON_DEFAULT_UPDATE_HZ,
                device.getDutyCycle(),
                device.getTorqueCurrent(),
                device.getMotorVoltage(),
                device.getPosition(),
                device.getVelocity(),
                device.getAcceleration());

        device.optimizeBusUtilization();
        return this;
    }

    private TalonFXDevice followWithRequest(int primaryDeviceID, boolean oppose) {
        device.setControl(new Follower(primaryDeviceID, oppose));
        return this;
    }

    private Supplier<Angle> replayPositionFromTalon() {
        MutAngle mutAngle = Units.Rotation.mutable(0);
        return () -> mutAngle.mut_setMagnitude(inputs.positionInputs.positionRotations);
    }

    public Supplier<Angle> positionSupplier(Frequency frequency) {
        return positionSupplier(frequency.in(Units.Hertz));
    }

    public Supplier<Angle> positionSupplier(double frequency) {
        MutAngle angle = Units.Rotations.mutable(0);

        return mapStatusSignalWithInputs(frequency, TalonFX::getPosition, inputs -> angle.mut_setMagnitude(inputs.positionInputs.positionRotations));
    }

    public Supplier<AngularVelocity> velocitySupplier(Frequency frequency) {
        return velocitySupplier(frequency.in(Units.Hertz));
    }

    public Supplier<AngularVelocity> velocitySupplier(double frequency) {
        MutAngularVelocity velocity = Units.RotationsPerSecond.mutable(0);
        return mapStatusSignalWithInputs(frequency, TalonFX::getVelocity, inputs -> velocity.mut_setMagnitude(inputs.positionInputs.velocityRotationsPerSec));
    }

    @Override
    protected TalonFXDevice getDeviceBuilderClass() {
        return this;
    }
}
