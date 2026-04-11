package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;

// TODO: Fix indentation inconsistencies
public class IntakePivot extends SubsystemBase implements Logged{
    private TalonFX motor;
    private Encoder encoder;
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private PIDController pid = new PIDController(Constants.IntakePivot.kP, Constants.IntakePivot.kI, Constants.IntakePivot.kD);
    private double defaultPosition;
    private boolean down;

    public IntakePivot() {
        SmartDashboard.putNumber("intake/Desired Pos", Constants.IntakePivot.OUT);
        this.motor = new TalonFX(Constants.IntakePivot.MOTOR_PORT);

        var request = new PositionDutyCycle(0.0);
        motor.setControl(request);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Voltage
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);

        motor.getConfigurator().apply(config);

        config.CurrentLimits
            .withStatorCurrentLimit(30)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLowerLimit(20)
            .withSupplyCurrentLimitEnable(true);

        defaultPosition = Constants.IntakePivot.ENCODER_UP;
    }

    // TODO: rename method and refactor
    public void intakePivot(double tick) {
        //tick = MathUtil.clamp(tick, -0.05, 0.05);

       // double currentCounts = encoder.get();
        double currentCounts = motor.getPosition().getValueAsDouble();
        dutyCycle.Output = pid.calculate(currentCounts, tick);

        motor.setControl(dutyCycle);
    }

    // where the intake should return when doing nothing
    public void setDefaultPosition(double defaultPosition) {
        this.defaultPosition = defaultPosition;
    }

    // where the intake should return when doing nothing
    public double getDefaultPosition() {
        return defaultPosition;
    }

    public void toggleDown() {
        down = !down;
    }

    public boolean getDown() {
        return down;
    }

    public void setPower(double armPower) {
        this.motor.set(armPower);
    }

    public Command moveOutCommand(){ // move the intake to the pos to fix up fuel
        return run( () -> intakePivot(Constants.IntakePivot.OUT));
    }

    public Command moveHomeCommand(){ // move home
        return run( () -> intakePivot(Constants.IntakePivot.HOME));
    }

    public double getEncoderValue() {
        return motor.getPosition().getValueAsDouble();
    }

    public boolean isDown() {
        return getEncoderValue() <= Constants.IntakePivot.ENCODER_DOWN_POINT_ELASTIC;
    }


  @Override
  public void periodic() {
    log("intake/intake encoder", motor.getPosition().getValueAsDouble());
    log("intake/Is intake out:", isDown());

    log("Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    log("Stator Current", motor.getStatorCurrent().getValueAsDouble());
    log("Velocity", motor.getVelocity().getValueAsDouble());
    log("Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
    log("Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
    log("Temperature", motor.getDeviceTemp().getValueAsDouble());
  }
}
