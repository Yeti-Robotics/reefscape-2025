package frc.robot.util.akit.device.can.cancolor;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutTemperature;
import frc.robot.util.akit.device.DeviceLogger;

public class CANColorDeviceLogger implements DeviceLogger<CANColorInputs> {
    private final Canandcolor canColor;
    private final MutTemperature temperature = Units.Celsius.mutable(0);

    public CANColorDeviceLogger(Canandcolor canColor) {
        this.canColor = canColor;
    }

    @Override
    public void updateInputs(CANColorInputs inputs) {
        inputs.temperature = temperature.mut_setMagnitude(canColor.getTemperature());

        ColorData colorData = canColor.getColor();

        inputs.promixity = canColor.getProximity();

        inputs.red = colorData.red();
        inputs.blue = colorData.blue();
        inputs.green = colorData.green();

        inputs.hue = colorData.hue();
        inputs.saturation = colorData.saturation();
        inputs.value = colorData.value();
    }
}
