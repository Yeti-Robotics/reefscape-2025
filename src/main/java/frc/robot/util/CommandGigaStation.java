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

    // BL1: 1st Row, Blue, Left
    // BR1: 1st Row, Blue, Right
    // WL1: 1st Row, White, Left
    // WR1: 1st Row, White, Right
    // GL1: 1st Row, Green, Left
    // GR1: 1st Row, Green, Right
    // BL2: 2nd Row, Blue, Left
    // BR2: 2nd Row, Blue, Right
    // WL2: 2nd Row, White, Left
    // WR2: 2nd Row, White, Right
    // GL2: 2nd Row, Green, Left
    // GR2: 2nd Row, Green, Right

    public Trigger bl1() {
        return button(GigaButtons.BL1.port);
    }
    public Trigger br1() {
        return button(GigaButtons.BR1.port);
    }
    public Trigger wl1() {
        return button(GigaButtons.WL1.port);
    }
    public Trigger wr1() {
        return button(GigaButtons.WR1.port);
    }
    public Trigger gl1() {
        return button(GigaButtons.GL1.port);
    }
    public Trigger gr1() {
        return button(GigaButtons.GR1.port);
    }

    public Trigger bl2() {
        return button(GigaButtons.BL2.port);
    }
    public Trigger br2() {
        return button(GigaButtons.BR2.port);
    }
    public Trigger wl2() {
        return button(GigaButtons.WL2.port);
    }
    public Trigger wr2() {
        return button(GigaButtons.WR2.port);
    }
    public Trigger gl2() {
        return button(GigaButtons.GL2.port);
    }
    public Trigger gr2() {
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
