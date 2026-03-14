// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class GrabFuel extends Command {
  IntakePivot intakePivot;
  PIDController pid;
  
  /** Creates a new GrabFuel. */
  public GrabFuel(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    this.pid = new PIDController(Constants.IntakePivot.SET_POINT_P,
                                 Constants.IntakePivot.SET_POINT_I,
                                 Constants.IntakePivot.SET_POINT_D);
    addRequirements( intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPos = intakePivot.getEncoderValue();
    double pidOutput = pid.calculate(currentPos, Constants.IntakePivot.ENCODER_DOWN);
    double clampedPid = MathUtil.clamp(pidOutput, -Constants.IntakePivot.SET_POINT_PID_CLAMP, Constants.IntakePivot.SET_POINT_PID_CLAMP);
    intakePivot.setPower(clampedPid); // may need to be negative
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
