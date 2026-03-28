// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
    private Shooter shooter;
    private Hood hood;
    private Supplier<Translation2d> targetSupplier;
    private double[] hoodAngleAndShooterPower = {-1, -1};

    public AutoAim(Shooter shooter, Hood hood, Supplier<Translation2d> targetSupplier) {
        this.shooter = shooter;
        this.hood = hood;
        this.targetSupplier = targetSupplier;
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        double distance = RobotContainer.turret.getAbsTurretPose()
            .getTranslation()
            .getDistance(targetSupplier.get());
        hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);
        shooter.setRPM(hoodAngleAndShooterPower[1]);
    }

    @Override
    public void execute() {
        double distance = RobotContainer.turret.getAbsTurretPose()
            .getTranslation()
            .getDistance(targetSupplier.get());
        hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);

        hood.goToAngle(hoodAngleAndShooterPower[0]);
        if (hood.atAngle(hoodAngleAndShooterPower[0])) {
            shooter.setRPM(hoodAngleAndShooterPower[1]);
        }

        SmartDashboard.putNumber("Shooter/distance from hub from autoAim", distance);
        SmartDashboard.putNumber("Shooter/RPM for auto aim", hoodAngleAndShooterPower[1]);
        SmartDashboard.putNumber("Shooter/hood for auto aim", hoodAngleAndShooterPower[0]);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        double target = hoodAngleAndShooterPower[1];
        double tolerance = target * Constants.Shooter.TOLERANCE;
        return shooter.getRPM() >= target - tolerance
            && shooter.getRPM() <= target + tolerance;
    }
}