package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Holds information about a simulated TalonFX. */
class TalonFXSimProfile extends PhysicsSim.SimProfile {
    private static final double MOTOR_RESISTANCE =
            0.002; // Assume 2mOhm resistance for voltage drop calculation

    private final DCMotorSim motorSim;
    private final TalonFXSimState talonFXSim;
    private CANcoderSimState cancoderSimState;

    /**
     * Creates a new simulation profile for a TalonFX device.
     *
     * @param talonFX The TalonFX device
     * @param rotorInertia Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final double rotorInertia) {
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this.motorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
        this.talonFXSim = talonFX.getSimState();
    }

    public TalonFXSimProfile(final TalonFX talonFX, final double rotorInertia, CANcoder cancoder) {
        this(talonFX, rotorInertia);
        cancoderSimState = cancoder.getSimState();
    }

    /**
     * Runs the simulation profile.
     *
     * <p>This uses very rudimentary physics simulation and exists to allow users to test features
     * of our products in simulation using our examples out of the box. Users may modify this to
     * utilize more accurate physics simulation.
     */
    public void run() {
        /// DEVICE SPEED SIMULATION

        motorSim.setInputVoltage(talonFXSim.getMotorVoltage());

        motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = motorSim.getAngularPositionRotations();
        final double velocity_rps =
                Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());

        if (cancoderSimState != null) {
            cancoderSimState.setRawPosition(position_rot);
            cancoderSimState.setVelocity(velocity_rps);
        }
        talonFXSim.setRawRotorPosition(position_rot);
        talonFXSim.setRotorVelocity(velocity_rps);

        talonFXSim.setSupplyVoltage(12 - talonFXSim.getSupplyCurrent() * MOTOR_RESISTANCE);
    }
}
