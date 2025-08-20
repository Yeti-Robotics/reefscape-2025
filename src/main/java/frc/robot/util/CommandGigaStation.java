package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandGigaStation extends CommandGenericHID {
    public enum GigaButtons {
        TOP_LEFT(1),
        TOP_MIDDLE(2),
        TOP_RIGHT(3),
        BOTTOM_LEFT(4),
        BOTTOM_MIDDLE(5),
        BOTTOM_RIGHT(6),
        BL1(7),
        BR1(8),
        WL1(9),
        WR1(10),
        GL1(11),
        GR1(12),
        BL2(13),
        BR2(14),
        WL2(15),
        WR2(16),
        GL2(17),
        GR2(18),
        AUTO1(19),
        AUTO2(20),
        AUTO3(21),
        AUTO4(22),
        AUTO5(23);

        public final int port;

        GigaButtons(int port) {
            this.port = port;
        }
    }

    public CommandGigaStation(int port) {
        super(port);
    }

    public Trigger topLeftSwitch() {
        return button(GigaButtons.TOP_LEFT.port);
    }

    public Trigger topMiddleSwitch() {
        return button(GigaButtons.TOP_MIDDLE.port);
    }

    public Trigger topRightSwitch() {
        return button(GigaButtons.TOP_RIGHT.port);
    }

    public Trigger bottomLeftSwitch() {
        return button(GigaButtons.BOTTOM_LEFT.port);
    }

    public Trigger bottomMiddleSwitch() {
        return button(GigaButtons.BOTTOM_MIDDLE.port);
    }

    public Trigger bottomRightSwitch() {
        return button(GigaButtons.BOTTOM_RIGHT.port);
    }

    /*
    +--------+---------+---------+---------+
    | Row    | Left    | Right   | Color   |
    +--------+---------+---------+---------+
    | 1st    | BL1     | BR1     | Blue    |
    | 1st    | WL1     | WR1     | White   |
    | 1st    | GL1     | GR1     | Green   |
    | 2nd    | BL2     | BR2     | Blue    |
    | 2nd    | WL2     | WR2     | White   |
    | 2nd    | GL2     | GR2     | Green   |
    +--------+---------+---------+---------+
    */

    public Trigger topLeftBlue() {
        return button(GigaButtons.BL1.port);
    }

    public Trigger topRightBlue() {
        return button(GigaButtons.BR1.port);
    }

    public Trigger topLeftWhite() {
        return button(GigaButtons.WL1.port);
    }

    public Trigger topRightWhite() {
        return button(GigaButtons.WR1.port);
    }

    public Trigger topLeftGreen() {
        return button(GigaButtons.GL1.port);
    }

    public Trigger topRightGreen() {
        return button(GigaButtons.GR1.port);
    }

    public Trigger bottomLeftBlue() {
        return button(GigaButtons.BL2.port);
    }

    public Trigger bottomRightBlue() {
        return button(GigaButtons.BR2.port);
    }

    public Trigger bottomLeftWhite() {
        return button(GigaButtons.WL2.port);
    }

    public Trigger bottomRightWhite() {
        return button(GigaButtons.WR2.port);
    }

    public Trigger bottomLeftGreen() {
        return button(GigaButtons.GL2.port);
    }

    public Trigger bottomRightGreen() {
        return button(GigaButtons.GR2.port);
    }

    public Trigger auto1() {
        return button(GigaButtons.AUTO1.port);
    }

    public Trigger auto2() {
        return button(GigaButtons.AUTO2.port);
    }

    public Trigger auto3() {
        return button(GigaButtons.AUTO3.port);
    }

    public Trigger auto4() {
        return button(GigaButtons.AUTO4.port);
    }

    public Trigger auto5() {
        return button(GigaButtons.AUTO5.port);
    }
}
