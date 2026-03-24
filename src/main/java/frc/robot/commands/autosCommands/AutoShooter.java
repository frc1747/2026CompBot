package frc.robot.commands.autosCommands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class AutoShooter extends Command{
        // shooting dir is froward.

    private TalonFX motorLeft;
    private TalonFX follower;
    private DutyCycleOut dutyCycleShooter = new DutyCycleOut(0);
    private VelocityVoltage velocityShooter = new VelocityVoltage(0.0).withSlot(0);
    private DutyCycleEncoder encoder;
    private final PIDController pid = new PIDController(Constants.Shooter.PID_P, Constants.Shooter.PID_I, Constants.Shooter.PID_D);
    private double desiredRPM = 0.0;
    public double desiredPower = 0.0;
    public TalonFXConfiguration configShooter;
    public double PID_P = Constants.Shooter.PID_P ;
    public double PID_I = Constants.Shooter.PID_I;
    public double PID_D = Constants.Shooter.PID_D;
    private Timer timer = new Timer();
    private double rpm;
    private double power;
    public AutoShooter(double power){
        this.power = power;
         motorLeft = new TalonFX(Constants.Shooter.MOTOR_LEFT_PORT);
        follower = new TalonFX(Constants.Shooter.MOTOR_RIGHT_PORT);
        dutyCycleShooter.Output = power;

        // config for the shooter motors
        configShooter = new TalonFXConfiguration();

        configShooter.Voltage
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        
        configShooter.Slot0.kP = Constants.Shooter.PID_P;
        configShooter.Slot0.kI = Constants.Shooter.PID_I;
        configShooter.Slot0.kD = Constants.Shooter.PID_D;

        configShooter.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motorLeft.getConfigurator().apply(configShooter);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        follower.getConfigurator().apply(configShooter);
        //motorLeft.getConfigurator().apply(configShooter);

        //pid.enableContinuousInput(0.0, 360.0);
        //pid.setTolerance(1.0);

        follower.setControl(new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        motorLeft.setControl(dutyCycleShooter);
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }
}