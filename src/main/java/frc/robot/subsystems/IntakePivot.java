package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    private TalonFX motor;
    private Encoder encoder;
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private PIDController pid = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);

    public IntakePivot() {
        this.motor = new TalonFX(Constants.IntakePivot.MOTOR_PORT);
        
        var request = new PositionDutyCycle(0.0);
        motor.setControl(request);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Voltage
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);

        motor.getConfigurator().apply(config);

    }

    public void intakePivot(double tick) {
        tick = MathUtil.clamp(tick, 0.0, Constants.Intake.INTAKE_PIVOT_TICK);
        
        double currentCounts = encoder.get();
        dutyCycle.Output = pid.calculate(currentCounts, tick);

        motor.setControl(dutyCycle);
    }

    public void setIntakePower(double armPower) {
        this.motor.set(armPower);
    }
}