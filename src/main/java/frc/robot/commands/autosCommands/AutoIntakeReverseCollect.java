package frc.robot.commands.autosCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeReverseCollect extends Command {
    private Intake intake;
    private double power;

    public AutoIntakeReverseCollect(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
    }

    @Override
    public void initialize() {
        intake.intakeSpin(power);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeSpin(0.0);
    }
}