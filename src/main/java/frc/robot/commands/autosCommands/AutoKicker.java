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
import frc.robot.subsystems.Kicker;

public class AutoKicker extends Command{

    private Kicker kicker;
    private Timer timer;

    public AutoKicker(Kicker kicker){
        this.timer = new Timer();
        this.kicker = kicker;
    }
    @Override
    public void initialize(){
        timer.reset();
        timer.start(); 
        kicker.setRPM(Constants.Kicker.MOTOR_RPM);
        
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        kicker.setPower(0);

    }
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }

}