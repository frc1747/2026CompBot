// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

// toggle between default position down and
// default position up on IntakePivot subsystem
public class ToggleIntakeReady extends InstantCommand {
    public IntakePivot intakePivot;

    public ToggleIntakeReady(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (intakePivot.getDown()) {
            intakePivot.setDefaultPosition(Constants.IntakePivot.ENCODER_UP);
        } else {
            intakePivot.setDefaultPosition(Constants.IntakePivot.ENCODER_READY);
        }
        intakePivot.toggleDown();
    }
}
