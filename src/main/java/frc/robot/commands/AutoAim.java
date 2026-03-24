// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
    private Shooter shooter;
    private Hood hood;
    private Pose2d target;
    private double[] hoodAngleAndShooterPower = {-1,-1};
    private double fudgeFactor;

    public AutoAim(Shooter shooter, Hood hood, double fudgeFactor ,Pose2d target) {
        this.shooter = shooter;
        this.hood = hood;
        this.target = target;
        this.fudgeFactor = fudgeFactor;
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double distance = RobotContainer.turret.getAbsTurretPose().getTranslation().getDistance(target.getTranslation());
        SmartDashboard.putNumber("Shooter/distance from hub from autoAim", distance);
        double[] hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);

        hood.goToAngleCommand(hoodAngleAndShooterPower[0]);
        if (hood.atAngle(hoodAngleAndShooterPower[0])){
            shooter.setRPM(hoodAngleAndShooterPower[1]*fudgeFactor);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        // better way of doing this idk 
        return (shooter.getRPM() <= hoodAngleAndShooterPower[1] + hoodAngleAndShooterPower[1]*Constants.Shooter.TOLERANCE && shooter.getRPM() >= hoodAngleAndShooterPower[1] - hoodAngleAndShooterPower[1]*Constants.Shooter.TOLERANCE);
    }
}
