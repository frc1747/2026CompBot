package frc.robot.subsystems;

import java.lang.invoke.ConstantCallSite;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    private TalonFX motor;
    private Encoder encoder;
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private PIDController pid = new PIDController(Constants.IntakePivot.kP, Constants.IntakePivot.kI, Constants.IntakePivot.kD);

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
        //tick = MathUtil.clamp(tick, -0.05, 0.05);
        
       // double currentCounts = encoder.get();
        double currentCounts = motor.getPosition().getValueAsDouble();
        dutyCycle.Output = pid.calculate(currentCounts, tick);

        motor.setControl(dutyCycle);
    }

    public void setPower(double armPower) {
        this.motor.set(armPower);
    }

    public Command moveOutCommand(){
        return run( () -> intakePivot(Constants.IntakePivot.OUT));
    }

    public Command moveHomeCommand(){
        return run( () -> intakePivot(Constants.IntakePivot.HOME));
    }

    
  @Override
  public void periodic() {
      SmartDashboard.putNumber("intake/intake encoder", motor.getPosition().getValueAsDouble());
  }
}