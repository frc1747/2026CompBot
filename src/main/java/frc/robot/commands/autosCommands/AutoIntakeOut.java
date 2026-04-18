package frc.robot.commands.autosCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class AutoIntakeOut extends Command {
    IntakePivot intakePivot;
    PIDController pid;
    private Timer timer = new Timer();

    public AutoIntakeOut(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
        this.pid = new PIDController(Constants.IntakePivot.SET_POINT_P,
                                    Constants.IntakePivot.SET_POINT_I,
                                    Constants.IntakePivot.SET_POINT_D);
        addRequirements( intakePivot);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        double currentPos = intakePivot.getEncoderValue();
        double pidOutput = pid.calculate(currentPos, Constants.IntakePivot.ENCODER_DOWN);
        double clampedPid = MathUtil.clamp(pidOutput, -Constants.IntakePivot.SET_POINT_PID_CLAMP, Constants.IntakePivot.SET_POINT_PID_CLAMP);
        intakePivot.setPower(clampedPid); // may need to be negati
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intakePivot.intakePivot(0);
        System.out.println("IntakeOut Has Been Ended");
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }
}
