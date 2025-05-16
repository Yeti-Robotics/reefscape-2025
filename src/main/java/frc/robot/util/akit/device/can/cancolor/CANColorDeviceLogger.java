package frc.robot.util.akit.device.can.cancolor;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.akit.device.DeviceLogger;

import static frc.robot.util.akit.device.can.CANUtil.CONNECTED_DEBOUNCE_TIME;

public class CANColorDeviceLogger implements DeviceLogger<CANColorInputs> {
    private final Canandcolor canColor;
    private final Debouncer connectedDebouncer = new Debouncer(CONNECTED_DEBOUNCE_TIME);

    public CANColorDeviceLogger(Canandcolor canColor) {
        this.canColor = canColor;
    }

    @Override
    public void updateInputs(CANColorInputs inputs) {
        inputs.isConnected = connectedDebouncer.calculate(canColor.isConnected());
        inputs.temperatureCelsius = canColor.getTemperature();

        ColorData colorData = canColor.getColor();

        inputs.proximity = canColor.getProximity();

        inputs.red = colorData.red();
        inputs.blue = colorData.blue();
        inputs.green = colorData.green();
    }
}
