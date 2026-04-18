// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.TargetPoses;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
    private Shooter shooter;
    private Hood hood;
    private double[] hoodAngleAndShooterPower = {-1,-1};

    public AutoAim(Shooter shooter) {
        this.shooter = shooter;
         // we default to blue like the cordnet system.
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
       // System.out.println("Shooter Initalized");

        // yes I am a hack
        double distance = RobotContainer.turret.getAbsTurretPose().getTranslation().getDistance(TargetPoses.getTargetPose().getTranslation());
        double[] hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);


        shooter.setRPM(hoodAngleAndShooterPower[1]);
    }

    @Override
    public void execute() {
        double distance = RobotContainer.turret.getAbsTurretPose().getTranslation().getDistance(TargetPoses.getTargetPose().getTranslation());
        double[] hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);

        //hood.goToAngleCommand(hoodAngleAndShooterPower[0]);
        if (RobotContainer.hood.atAngle(hoodAngleAndShooterPower[0])){
            shooter.setRPM(hoodAngleAndShooterPower[1]);
        }
        SmartDashboard.putNumber("Shooter/distance from hub from autoAim", distance);
        SmartDashboard.putNumber("Shooter/RPM for auto aim", hoodAngleAndShooterPower[1]);
        SmartDashboard.putNumber("Shooter/hood for auto aim", hoodAngleAndShooterPower[0]);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        // better way of doing this idk
        return false;
    }
}
