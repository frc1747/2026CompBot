package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

    private TalonFX motor;
    private VelocityVoltage velocityKicker = new VelocityVoltage(0).withSlot(0);
    private DutyCycleOut dutyControl = new DutyCycleOut(0.0);
    private double desiredRPM = 0.0;

    public Kicker() {
        motor = new TalonFX(Constants.Kicker.MOTOR_PORT);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Voltage
            .withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0);

        config.Slot0.kP = 0.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        config.CurrentLimits
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLimitEnable(true);

        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
                
        motor.getConfigurator().apply(config);
        motor.getStatorCurrent().setUpdateFrequency(50);
        motor.getSupplyVoltage().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(50);
        motor.getSupplyCurrent().setUpdateFrequency(50);
        motor.optimizeBusUtilization();
    }

    public Command run() {
        return runOnce(() -> setPower(Constants.Kicker.MOTOR_POWER));
    }

    public Command stop() {
        return runOnce(() -> setPower(0.0));
    }
    
    public void setPower(double power) {
        dutyControl.Output = power;
        motor.setControl(dutyControl);
    }

    public Command setSpeedToDesired() {
        return runOnce(() -> setRPM(desiredRPM));
    }

    public void setRPM(double rpm) {
        motor.setControl(velocityKicker.withVelocity(rpm / 60.0));
    }

    public double getRPM() {
        return motor.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public void periodic() {

    }
}
