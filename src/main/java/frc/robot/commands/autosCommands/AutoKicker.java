package frc.robot.commands.autosCommands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


import frc.robot.Constants;

public class AutoKicker extends Command{

    private TalonFX motor;
    private VelocityVoltage velocityKicker = new VelocityVoltage(0).withSlot(0);
    private DutyCycleOut dutyControl = new DutyCycleOut(0.0);
    private double rpm;
    private double power;
    private Timer timer = new Timer();

    public AutoKicker(double power){
        this.power = power;
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

        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
                
        motor.getConfigurator().apply(config);
        motor.getStatorCurrent().setUpdateFrequency(50);
        motor.getSupplyVoltage().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(50);
        motor.getSupplyCurrent().setUpdateFrequency(50);
        motor.optimizeBusUtilization();
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start(); 
        dutyControl.Output = power;
        motor.setControl(dutyControl);
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        dutyControl.Output = 0.0;
        motor.setControl(dutyControl);

    }
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }

}