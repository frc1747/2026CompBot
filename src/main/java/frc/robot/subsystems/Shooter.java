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
import frc.robot.RobotContainer;

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

    public double getRPMNeededFromDistanceAndAngle(double x, double y){
        // Z = A + BX + CY + DX^2 + FY^2 + EXY is the quady E Z is rpm, X is distance, Y is hood angle
        return (Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B*x + Constants.Shooter.SURFACE_C*y + Constants.Shooter.SURFACE_D*Math.pow(x,2) + Constants.Shooter.SURFACE_F*Math.pow(y,2) + Constants.Shooter.SURFACE_E*x*y)/100;
    }

    public double getDistanceNeededFromAngleAndRPM(double y, double z ){
        double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_C*y + Constants.Shooter.SURFACE_F*Math.pow(y,2) +- z*100;
        double B =  Constants.Shooter.SURFACE_B + Constants.Shooter.SURFACE_E;
        double A = Constants.Shooter.SURFACE_D;
        double aws = (- B + Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A; // we need to see if it's postive or negative
        if (aws > 0) return aws;
        return (- B - Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A;
        // slove with the good old quady form
    }

    public double getAngleNeededFromDistanceAndRPM(double x, double z ){
        double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B*x + Constants.Shooter.SURFACE_D*Math.pow(x,2) +- z*100;
        double B =  Constants.Shooter.SURFACE_C + Constants.Shooter.SURFACE_E;
        double A = Constants.Shooter.SURFACE_F;
        double aws = (- B + Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A; // we need to see if it's postive or negative
        if (aws > 0) return aws;
        return (- B - Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A;
        // slove with the good old quady form
    }

    public double[] findSpeedAndAngleFromDistance(double Distance){
        // the power is multplyed by 100 to move up to the thousands
        // better search needed
        double currentAngle = RobotContainer.hood.getCurrentAngle();
        double wantedRPM = getRPMNeededFromDistanceAndAngle(Distance, currentAngle);
        double[] array = {-1,-1};
        // this could be refactor 
        if (wantedRPM <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
            double[] angleAndSpeed = {currentAngle, wantedRPM*Constants.Shooter.AUTO_SHOOTER_MULT};
            return angleAndSpeed;
        }

        // we are assuming that greater hood angle is a furtuer Shoot
        for (currentAngle = RobotContainer.hood.getCurrentAngle() ; currentAngle <= Constants.Shooter.MAX_HOOD_ANGLE ; currentAngle ++ ){
            if (currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE) return array;
            if (wantedRPM <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
                double[] angleAndSpeed = {currentAngle, wantedRPM*Constants.Shooter.AUTO_SHOOTER_MULT};
                return angleAndSpeed;
            }
        }

        for (currentAngle = RobotContainer.hood.getCurrentAngle() ; currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE ; currentAngle -- ){
            if (currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE) return array;
            if (wantedRPM <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
                double[] angleAndSpeed = {currentAngle, wantedRPM*Constants.Shooter.AUTO_SHOOTER_MULT};
                return angleAndSpeed;
            }
        }
        return array;
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
