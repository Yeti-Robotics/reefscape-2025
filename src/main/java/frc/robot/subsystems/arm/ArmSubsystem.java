package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX talon = new TalonFX(ArmConstants.ARM_CAN_ID, ArmConstants.ARM_CAN_BUS);

    private final SysIdRoutine routine;

    public ArmSubsystem() {
        StatusSignal<Voltage> motorVoltage = talon.getMotorVoltage();
        StatusSignal<Angle> motorRotations = talon.getRotorPosition();
        StatusSignal<AngularVelocity> motorRotationsVelo = talon.getRotorVelocity();
        StatusSignal<AngularAcceleration> accelerationStatusSignal = talon.getAcceleration();
        StatusSignal<Current> supplyCurrent = talon.getSupplyCurrent();
        StatusSignal<Current> statorCurrent = talon.getStatorCurrent();
        this.routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(this::applyOutput,
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
                                    .angularAcceleration(accelerationStatusSignal.getValue())
                                    .current(supplyCurrent.getValue())
                                    .value("Stator current", statorCurrent.getValueAsDouble(), "Amps");
                        },
                        this)
        );
    }

    public void applyOutput(Voltage output) {
        double val = output.in(Volts);
        MotionMagicTorqueCurrentFOC req = new MotionMagicTorqueCurrentFOC(0.036)
                .withSlot(2)
                .withFeedForward(val);
        talon.setControl(req);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
