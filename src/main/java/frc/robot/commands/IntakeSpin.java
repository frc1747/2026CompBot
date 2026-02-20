package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeSpin extends Command {
    private Intake intake;
    private double power;

    public IntakeSpin(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intakeSpin(power);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeSpin(0.0);
    }
}