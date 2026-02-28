// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private Shooter shooter;
  private Hood hood;
  private Pose2d target;
  private double[] hoodAngleAndShooterPower = {-1,-1};

  public AutoAim(Shooter shooter, Hood hood, Pose2d target) {
    this.shooter = shooter;
    this.hood = hood;
    this.target = target;
    addRequirements(shooter, hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = RobotContainer.turret.getAbsTurretPose().getTranslation().getDistance(target.getTranslation());
    double[] hoodAngleAndShooterPower = shooter.findSpeedAndAngleFromDistance(distance);

    hood.goToAngleCommand(hoodAngleAndShooterPower[0]);
    if (hood.atAngle(hoodAngleAndShooterPower[0])){
      shooter.setRPM(hoodAngleAndShooterPower[1]);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // better way of doing this idk 
    return (shooter.getRPM() <= hoodAngleAndShooterPower[1] + hoodAngleAndShooterPower[1]*Constants.Shooter.TOLERANCE && shooter.getRPM() >= hoodAngleAndShooterPower[1] - hoodAngleAndShooterPower[1]*Constants.Shooter.TOLERANCE);
  }
}
