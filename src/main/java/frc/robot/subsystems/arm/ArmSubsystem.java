package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX talon = new TalonFX(ArmConstants.ARM_CAN_ID, ArmConstants.ARM_CAN_BUS);

    private final SysIdRoutine routine;

    StatusSignal<Angle> motorRotations = talon.getRotorPosition();

    @Override
    public void periodic() {
        motorRotations.refresh();
    }

    public ArmSubsystem() {
        StatusSignal<Voltage> motorVoltage = talon.getMotorVoltage();
        StatusSignal<AngularVelocity> motorRotationsVelo = talon.getRotorVelocity();
        StatusSignal<AngularAcceleration> accelerationStatusSignal = talon.getAcceleration();
        StatusSignal<Current> supplyCurrent = talon.getSupplyCurrent();
        StatusSignal<Current> statorCurrent = talon.getStatorCurrent();
        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.of(4).per(Second),
                Volts.of(22),
                Time.ofBaseUnits(15, Seconds)
        );


        this.routine =
                new SysIdRoutine(config,
                        new SysIdRoutine.Mechanism(
                                this::applyOutput,
                                log -> {
                                    motorVoltage.refresh();
                                    motorRotations.refresh();
                                    motorRotationsVelo.refresh();
                                    accelerationStatusSignal.refresh();
                                    supplyCurrent.refresh();
                                    statorCurrent.refresh();
                                    log.motor("arm_motor")
                                            .voltage(motorVoltage.getValue())
                                            .angularPosition(motorRotations.getValue())
                                            .angularVelocity(motorRotationsVelo.getValue())
                                            .angularAcceleration(
                                                    accelerationStatusSignal.getValue())
                                            .current(supplyCurrent.getValue())
                                            .value(
                                                    "Stator current",
                                                    statorCurrent.getValueAsDouble(),
                                                    "Amps");
                                },
                                this));
    }

    public void applyOutput(Voltage output) {
        TorqueCurrentFOC req =
                new TorqueCurrentFOC(output.in(Volts));
        talon.setControl(req);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public double getPosition() {
        return motorRotations.getValueAsDouble();
    }
}
