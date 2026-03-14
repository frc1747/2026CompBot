package frc.robot.commands.AutosCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakePivot;

public class AutoIntakeOut extends InstantCommand {
    private IntakePivot intakePivot;
    private int tick;

    public AutoIntakeOut(IntakePivot intakePivot, int tick) {
        this.intakePivot = intakePivot;
        this.tick = tick;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        intakePivot.intakePivot(tick);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.intakePivot(0);
    } 
}