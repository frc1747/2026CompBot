package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakeOut extends Command {
    private IntakePivot intakePivot;
    private double power;

    public IntakeOut(IntakePivot intakePivot, double power) {
        this.intakePivot = intakePivot;
        this.power = power;
        addRequirements(intakePivot);
    }

    @Override
    public void execute() {
        intakePivot.intakePivot(power);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.intakePivot(0.0);
    } 

}