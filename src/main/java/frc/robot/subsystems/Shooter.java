// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    // shooting dir is froward.
    private TalonFX motorLeft;
    private TalonFX follower;
    private VelocityVoltage velocityShooter = new VelocityVoltage(0.0).withSlot(0);
    private double desiredRPM = 0.0;
    // public double desiredPower = 0.0;
    public TalonFXConfiguration configShooter;
    public double PID_P = Constants.Shooter.PID_P ;
    public double PID_I = Constants.Shooter.PID_I;
    public double PID_D = Constants.Shooter.PID_D;
    private double shooterOffset = 0.0;


    public Shooter() {
        motorLeft = new TalonFX(Constants.Shooter.MOTOR_LEFT_PORT);
        follower = new TalonFX(Constants.Shooter.MOTOR_RIGHT_PORT);

        // config for the shooter motors
        configShooter = new TalonFXConfiguration();

        configShooter.Voltage
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);

        configShooter.Slot0.kP = PID_P;
        configShooter.Slot0.kI = PID_I;
        configShooter.Slot0.kD = PID_D;

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

        follower.setControl(new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        SmartDashboard.putNumber("Shooter/Desired RPM", 0.0);
        SmartDashboard.putNumber("Shooter/Shooter pid P", PID_P);
        SmartDashboard.putNumber("Shooter/Shooter pid I", PID_I);
        SmartDashboard.putNumber("Shooter/Shooter pid D", PID_D);
    }

    public Command stopCommand() {
        return runOnce(() -> setRPM(0.0));
    }

    public Command offsetDecrement() {
        return run(() -> shooterOffset -= Constants.Shooter.AUTOSHOOT_OFFSET_INCREMENT);
    }

    public Command offsetIncrement() {
        return run(() -> shooterOffset += Constants.Shooter.AUTOSHOOT_OFFSET_INCREMENT);
    }

    // in RPM
    public void setRPM(double rpm) {
        // Downstep revolutions per second to revolutions per minute
        motorLeft.setControl(velocityShooter.withVelocity((rpm + shooterOffset) / 60.0));
    }

    public double getRPM() {
        // Upstep revolutions per second to revolutions per minute
        return (motorLeft.getVelocity().getValueAsDouble() + follower.getVelocity().getValueAsDouble()) / 2 * 60;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Average RPM", getRPM());
        desiredRPM = SmartDashboard.getNumber("Shooter/Desired RPM", 0.0);
        //SmartDashboard.putNumber("Shooter/Desired RPM error", Math.abs((desiredRPM - getRPM()) / getRPM()));
        if (Math.abs((desiredRPM - getRPM()) / getRPM()) <= 0.01) {
            SmartDashboard.putBoolean("Shooter/Desired RPM Reached", true);
        } else{
            SmartDashboard.putBoolean("Shooter/Desired RPM Reached", false);
        }
        desiredRPM = SmartDashboard.getNumber("Shooter/Desired RPM", 0.0) ;
        SmartDashboard.putNumber("number I am putting on smartdashbard", desiredRPM);

        //setRPM(desiredRPM);
        SmartDashboard.putNumber("shooter/PID", velocityShooter.withVelocity(desiredRPM/60).Velocity);
        SmartDashboard.getNumber("Shooter/Desired RPM?", desiredRPM) ;

        // auto shoot


        // pids yay!!!!
        PID_P = SmartDashboard.getNumber("Shooter/Shooter pid P", PID_P);
        PID_I = SmartDashboard.getNumber("Shooter/Shooter pid I", PID_I);
        PID_D = SmartDashboard.getNumber("Shooter/Shooter pid D", PID_D);
        SmartDashboard.putNumber("Shooter/Shooter true pid P",  configShooter.Slot0.kP);

        if(configShooter.Slot0.kP != PID_P)  {
            configShooter.Slot0.kP = PID_P;
        }

        if(configShooter.Slot0.kI != PID_I)  {
            configShooter.Slot0.kI = PID_I;
        }

        if(configShooter.Slot0.kD != PID_D)  {
            configShooter.Slot0.kD = PID_D;
        }
    }
}
