package frc.robot.commands.AutosCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeCollect extends Command {
    private Intake intake;
    private double power;

    public AutoIntakeCollect(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intakeSpin(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeSpin(0.0);
    }
}