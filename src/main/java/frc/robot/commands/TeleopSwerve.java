// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopSwerve extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final DoubleSupplier rotationSup;
  private final FieldCentric swerveRequest;
  
  public TeleopSwerve(CommandSwerveDrivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    this.drivetrain = drivetrain;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.swerveRequest = new FieldCentric();
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.setControl(
      swerveRequest
        .withVelocityX(-translationSup.getAsDouble() * Constants.Drivetrain.MAX_SPEED)
        .withVelocityY(-strafeSup.getAsDouble() * Constants.Drivetrain.MAX_SPEED)
        .withRotationalRate(-rotationSup.getAsDouble() * Constants.Drivetrain.MAX_ANGULAR_VELOCITY)
    );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
      swerveRequest
        .withVelocityX(0.0)
        .withVelocityY(0.0)
        .withRotationalRate(0.0)
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
