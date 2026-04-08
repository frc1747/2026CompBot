package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;


public class AutoIntakeStash extends Command{
    private IntakePivot intakePivot;
    private Timer timer = new Timer();
    PIDController pid;

    public AutoIntakeStash(IntakePivot intakePivot){
        this.intakePivot = intakePivot;
        this.pid = new PIDController(Constants.IntakePivot.SET_POINT_P,
                                    Constants.IntakePivot.SET_POINT_I,
                                    Constants.IntakePivot.SET_POINT_D);
        addRequirements(intakePivot);
    }


    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        double currentPos = intakePivot.getEncoderValue();
        double pidOutput = pid.calculate(currentPos, intakePivot.getDefaultPosition());
        double clampedPid = MathUtil.clamp(pidOutput, -Constants.IntakePivot.SET_POINT_PID_CLAMP, Constants.IntakePivot.SET_POINT_PID_CLAMP);
        intakePivot.setPower(clampedPid);
    }

    @Override 
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        intakePivot.setPower(0.0);
        System.out.println("IntakeStash Has Been Ended");
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3.0);
    }
}
