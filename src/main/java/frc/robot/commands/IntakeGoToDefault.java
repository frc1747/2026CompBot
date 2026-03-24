// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

// may want to make this a default command to have always running,
// tring to reach and stay at default setpoint until a different
// command takes over. If that approach is taken, isFinished should
// be modified so that it always returns false
public class IntakeGoToDefault extends Command {
    IntakePivot intakePivot;
    PIDController pid;

    /** Creates a new RaiseIntake. */
    public IntakeGoToDefault(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
        this.pid = new PIDController(Constants.IntakePivot.SET_POINT_P,
                                    Constants.IntakePivot.SET_POINT_I,
                                    Constants.IntakePivot.SET_POINT_D);
        addRequirements(intakePivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentPos = intakePivot.getEncoderValue();
        double pidOutput = pid.calculate(currentPos, intakePivot.getDefaultPosition());
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
        // if (Math.abs(intakePivot.getEncoderValue() - intakePivot.getDefaultPosition()) <= Constants.IntakePivot.SET_POINT_TOLERANCE) {
        //   return true;
        // } else {
        //   return false;
        // }
    }
}
