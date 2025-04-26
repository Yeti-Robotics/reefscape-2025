package frc.robot.util.akit.device.can.cancolor;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import frc.robot.util.akit.device.can.CANDeviceBuilder;
import frc.robot.util.akit.device.can.CANUtil;

public class CANColorDevice extends CANDeviceBuilder<Canandcolor, CanandcolorSettings, CANColorInputs, CANColorDevice> {
    private CANColorDevice(Canandcolor device) {
        super(device);
    }

    public static CANColorDevice configure(int id) {
        return from(new Canandcolor(id));
    }

    public static CANColorDevice from(Canandcolor canandcolor) {
        return new CANColorDevice(canandcolor);
    }

    @Override
    protected CANColorDevice getDeviceBuilderClass() {
        return this;
    }

    @Override
    public CANColorDevice syncConfigs() {
        CANUtil.tryUntilOk(() -> getDevice().setSettings(getConfig()));
        return this;
    }

    @Override
    protected CANColorInputs createDeviceInputs() {
        return new CANColorInputsAutoLogged();
    }

    @Override
    protected CANColorDeviceLogger getLogger() {
        return new CANColorDeviceLogger(getDevice());
    }

    @Override
    protected CanandcolorSettings getDefaultConfig() {
        return new CanandcolorSettings();
    }
}
