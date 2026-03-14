package frc.robot.commands.AutosCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class AutoIntakeCollect extends InstantCommand {
    private Intake intake;
    private double power;

    public AutoIntakeCollect(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
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