package frc.robot.util.sim;

import java.util.ArrayList;

public class MechanismSimManager {
    private static MechanismSimManager instance;
    private static final ArrayList<Simulatable> mechanisms = new ArrayList<>();

    private MechanismSimManager() {}

    public static void addMechanism(Simulatable mechanism) {
        mechanisms.add(mechanism);
    }

    public static void updateMechanisms() {
        mechanisms.forEach(Simulatable::update);
    }
}
