package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;


public class AutoIntakeStash extends Command{
    private IntakePivot intakePivot;
    private Timer timer = new Timer();

    public AutoIntakeStash(IntakePivot intakePivot){
        this.intakePivot = intakePivot;
        addRequirements(intakePivot);
    }


    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    //     if (intakePivot.getDown()) {
    //         intakePivot.setDefaultPosition(Constants.IntakePivot.ENCODER_UP);
    //     } else {
    //         intakePivot.setDefaultPosition(Constants.IntakePivot.ENCODER_READY);
    //     }
    //    intakePivot.toggleDown();
        intakePivot.setPower(0.35);
        intakePivot.intakePivot(Constants.IntakePivot.HOME);
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
        return timer.hasElapsed(0.5);
    }
}
