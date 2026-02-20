package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakeOut extends Command {
    private IntakePivot intakePivot;
    private int tick;

    public IntakeOut(IntakePivot intakePivot, int tick) {
        this.intakePivot = intakePivot;
        this.tick = tick;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakePivot.intakePivot(tick);
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.intakePivot(0);
    } 
}