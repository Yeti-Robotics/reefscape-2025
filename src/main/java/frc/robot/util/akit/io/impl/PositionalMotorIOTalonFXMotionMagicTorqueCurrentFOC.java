package frc.robot.util.akit.io.impl;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;

public class PositionalMotorIOTalonFXMotionMagicTorqueCurrentFOC extends PositionalMotorIOTalonFX {
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);

    public PositionalMotorIOTalonFXMotionMagicTorqueCurrentFOC(TalonFX talon) {
        super(talon);
    }

    @Override
    public void setPosition(Angle value) {
        talon.setControl(motionMagicTorqueCurrentFOC.withPosition(value));
    }
}
