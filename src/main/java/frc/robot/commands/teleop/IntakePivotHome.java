// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePivotHome extends Command {
  /** Creates a new IntakePivotHome. */
  private IntakePivot intakePivot;

  public IntakePivotHome(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePivot.intakePivot(Constants.IntakePivot.HOME);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = intakePivot.getIntakePivotAngle();
    if (currentAngle <= Constants.IntakePivot.HOME + Constants.IntakePivot.HOME * Constants.IntakePivot.TOLERANCE && currentAngle >= Constants.IntakePivot.HOME - Constants.IntakePivot.HOME * Constants.IntakePivot.TOLERANCE) {
      return true;
    }

    return false;
  }
}
