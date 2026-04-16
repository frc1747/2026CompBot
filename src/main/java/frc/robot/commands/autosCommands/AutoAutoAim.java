// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.autosCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class AutoAutoAim extends Command {
    private Shooter shooter;
    private Hood hood;
    private Pose2d target;
    private double[] hoodAngleAndShooterPower = {-1,-1};
    private Timer timer = new Timer();

    public AutoAutoAim(Shooter shooter, Hood hood) {
        this.shooter = shooter;
        this.hood = hood;
        this.target = Constants.Shooter.BLUE_HUB_CENTER_POSE2D; // we default to blue like the cordnet system.
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // // yes I am a hack
        double distance = RobotContainer.turret.getAbsTurretPose().getTranslation().getDistance(target.getTranslation());
        double[] hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);

        hood.goToAngleCommand(hoodAngleAndShooterPower[0]);
        if (hood.atAngle(hoodAngleAndShooterPower[0])){
            shooter.setRPM(hoodAngleAndShooterPower[1]);
        }
       // shooter.setPower(0.3);
    }

    @Override
    public void execute() {
    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    //         this.target = Constants.Shooter.RED_HUB_CENTER_POSE2D;
    //     }
    //     double distance = RobotContainer.turret.getAbsTurretPose().getTranslation().getDistance(target.getTranslation());
    //     double[] hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);

    //     hood.goToAngleCommand(hoodAngleAndShooterPower[0]);
    //     if (hood.atAngle(hoodAngleAndShooterPower[0])){
    //         shooter.setRPM(hoodAngleAndShooterPower[1]);
    //     }
    //     SmartDashboard.putNumber("Shooter/distance from hub from autoAim", distance);
    //     SmartDashboard.putNumber("Shooter/RPM for auto aim", hoodAngleAndShooterPower[1]);
    //     SmartDashboard.putNumber("Shooter/hood for auto aim", hoodAngleAndShooterPower[0]);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0.0);
        System.out.println("Shooter Has Been Stopped");
    }

    @Override
    public boolean isFinished() {
        // better way of doing this idk
        return timer.hasElapsed(6.0);
    }
}
