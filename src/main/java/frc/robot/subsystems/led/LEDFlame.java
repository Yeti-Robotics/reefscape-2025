package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.Random;

public class LEDFlame {
    private final AddressableLED led;
    private final AddressableLEDBuffer store;
    private final Random random = new Random();

    public LEDFlame(AddressableLED led, AddressableLEDBuffer buffer) {
        this.led = led;
        this.store = buffer;
    }

    // this is the edge case, for when no LEDs
    public void updateFlame() {
        int length = store.getLength();
        if (length <= 1) {
            return;
        }

        for (int i = 0; i < length; i++) {
            int maxIntense = 16;
            int intensity = random.nextInt(maxIntense + 1);

            int red = (int) (intensity * LEDConstants.CRISP_WHITE.red / maxIntense);
            int green = (int) (intensity * LEDConstants.CRISP_WHITE.green / maxIntense);
            int blue = (int) (intensity * LEDConstants.YETI_BLUE.blue / maxIntense);

            store.setRGB(i, red, green, blue);
        }
        led.setData(store);
    }
}
