package frc.robot.commands.autosCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class AutoIntakeOut extends Command {
    private IntakePivot intakePivot;
    private int tick;

    public AutoIntakeOut(IntakePivot intakePivot, int tick) {
        this.intakePivot = intakePivot;
        this.tick = tick;

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
    public boolean isFinished() {
        return false;
    }
}