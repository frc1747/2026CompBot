// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N20;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  // shooting dir is froward.
  private TalonFX motorLeft;
  private TalonFX follower;
  private DutyCycleOut dutyCycleShooter = new DutyCycleOut(0);
  private VelocityVoltage velocityShooter = new VelocityVoltage(0.0).withSlot(0);
  private DutyCycleEncoder encoder;
  private final PIDController pid = new PIDController(Constants.Shooter.PID_P, Constants.Shooter.PID_I, Constants.Shooter.PID_D);
  private double desiredRPM = 0.0;
  public double desiredPower = 0.0;
  private Nat<N6> dergee = Nat.N6();
  Matrix<N6,N1> martrix;

  public Shooter() {
    motorLeft = new TalonFX(Constants.Shooter.MOTOR_LEFT_PORT);
    follower = new TalonFX(Constants.Shooter.MOTOR_RIGHT_PORT);

    // config for the shooter motors
    TalonFXConfiguration configShooter = new TalonFXConfiguration();

    configShooter.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12);
    
    configShooter.Slot0.kP = Constants.Shooter.PID_P;
    configShooter.Slot0.kI = Constants.Shooter.PID_I;
    configShooter.Slot0.kD = Constants.Shooter.PID_D;

    configShooter.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motorLeft.getConfigurator().apply(configShooter);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    follower.getConfigurator().apply(configShooter);
    //motorLeft.getConfigurator().apply(configShooter);

    //pid.enableContinuousInput(0.0, 360.0);
    //pid.setTolerance(1.0);

    follower.setControl(new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    SmartDashboard.putNumber("Shooter/Desired RPM", 0.0);
    SmartDashboard.putNumber("Shooter/Desired Power", 0.0);
    double[][] listOfPoints = {{1,2,3,4,5,6,10,8,9},{1,4,9,16,25,36,49,64,81}};

    Matrix<N6,N1> martrix = GetCoeffients(listOfPoints);
  }

  public Command setPowerCommand(double power){
    return runOnce(() -> setPower(power));
  }

  public Command SetDesiredPowerCommand(){
    return run(() -> setPower(desiredPower));
  }

  public Command stopCommand() {
    return runOnce(() -> setPower(0.0));
  }

  public Command setSpeedToDesired() {
    return run(() -> setRPM(desiredRPM));
  }

  public void setPower(double power) {
    dutyCycleShooter.Output = power;
    motorLeft.setControl(dutyCycleShooter);
  }

  // in RPM
  public void setRPM(double rpm) {
    System.out.println("I am being commanded to " + rpm);
    //velocityShooter.Velocity * 60 = //pid.calculate(getRPM(), rpm);
    motorLeft.setControl(velocityShooter.withVelocity(rpm/60.0));
  }

  public double getRPM() {
    return (motorLeft.getVelocity().getValueAsDouble() + follower.getVelocity().getValueAsDouble()) / 2 * 60;
  }

  public double getPowerNeededFromDistanceAndAngle(double x, double y){
    // Z = A + BX + CY + DX^2 + FY^2 + EXY is the quady E Z is power, X is distance, Y is hood angle
    return (Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B*x + Constants.Shooter.SURFACE_C*y + Constants.Shooter.SURFACE_D*Math.pow(x,2) + Constants.Shooter.SURFACE_F*Math.pow(y,2) + Constants.Shooter.SURFACE_E*x*y)/100;
  }

  public double getdistanceNeededFromAngleAndPower(double y, double z ){
    double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_C*y + Constants.Shooter.SURFACE_F*Math.pow(y,2) +- z*100;
    double B =  Constants.Shooter.SURFACE_B + Constants.Shooter.SURFACE_E;
    double A = Constants.Shooter.SURFACE_D;
    double aws = (- B + Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A; // we need to see if it's postive or negative
    if (aws > 0) return aws;
    return (- B - Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A;
    // slove with the good old quady form
  }

  public double getAngleNeededFromDistanceAndPower(double x, double z ){
    double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B*x + Constants.Shooter.SURFACE_D*Math.pow(x,2) +- z*100;
    double B =  Constants.Shooter.SURFACE_C + Constants.Shooter.SURFACE_E;
    double A = Constants.Shooter.SURFACE_F;
    double aws = (- B + Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A; // we need to see if it's postive or negative
    if (aws > 0) return aws;
    return (- B - Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A;
    // slove with the good old quady form
  }

  public double[] findSpeedAndAngleFromDistance(double Distance){
    double currentAngle = RobotContainer.hood.getCurrentAngle();
    double wantedPower = getPowerNeededFromDistanceAndAngle(Distance, currentAngle);
    double[] array = {-1,-1};
    // this could be refactor 
    if (wantedPower <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
     double[] angleAndSpeed = {currentAngle, wantedPower};
      return angleAndSpeed;
  }
  // we are assuming that greater hood angle is a furtuer Shoot
    for (currentAngle = RobotContainer.hood.getCurrentAngle() ; currentAngle <= Constants.Shooter.MAX_HOOD_ANGLE ; currentAngle ++ ){
      if (currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE) return array;
      if (wantedPower <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
        double[] angleAndSpeed = {currentAngle, wantedPower};
        return angleAndSpeed;}
    }
    for (currentAngle = RobotContainer.hood.getCurrentAngle() ; currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE ; currentAngle -- ){
      if (currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE) return array;
      if (wantedPower <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
        double[] angleAndSpeed = {currentAngle, wantedPower};
        return angleAndSpeed;}
    }
    return array;

  }

  private double SumOfPoints(double[] listOfPoints){
    double total = 0;
    for (int i = 0; i < listOfPoints.length ; i ++ ){
      total += listOfPoints[i];
    }
    return total;
  }

  private Matrix<N6,N6> MatrixOfSumOfXpoints(double[][] listOfPoints){
    Matrix<N6,N6> matrix = new Matrix<>(Nat.N6(), Nat.N6() );
    System.out.print(matrix.getNumCols() );
    for (int i = 0 ; i < matrix.getNumCols(); i++){

      for (int j = 0 ; j < matrix.getNumCols(); j++){
        // soo how this works is we are sending all of the x values of because the array is set up like this
        // [ x0 ]   [ y0 ]
        // [ x1 ]   [ y1 ]
        // [ x2 ]   [ y2 ]
        // [ x3 ] , [ y3 ]
        // by sending the array of points
        matrix.set(i,j,
        Math.pow(SumOfPoints(listOfPoints[0]),j+i));
      }
      
    }
    return matrix;
  }

  private Matrix<N6,N1> MatrixOfSumOfYXpoints(double[][] listOfPoints){
    double total = 0;
    Matrix<N6,N1> matrix = new Matrix<N6,N1>(Nat.N6(),Nat.N1());
    for(int i = 0 ; i < matrix.getNumRows(); i++){
      total = 0;
      for(int j = 0; j < listOfPoints[0].length; j++ ){
        total += listOfPoints[0][j] + listOfPoints[1][j];
      }
      matrix.set(i,0,total);
    }
    return matrix;
  }

  private Matrix<N6,N1> GetCoeffients(double[][] listOfPoints){
    System.out.print("inv matrix" + MatrixOfSumOfXpoints(listOfPoints) + "dem" + MatrixOfSumOfXpoints(listOfPoints).det());
    return MatrixOfSumOfXpoints(listOfPoints).solve(MatrixOfSumOfYXpoints(listOfPoints));
  }


  @Override
  public void periodic() {
    
    System.out.print(martrix.get(1,1) + " " + martrix.get(2,1) + " " + martrix.get(3,1) + " " + martrix.get(4,1) + " " + martrix.get(5,1) );
    SmartDashboard.putNumber("Shooter/Average RPM", getRPM());
    desiredRPM = SmartDashboard.getNumber("Shooter/Desired RPM", 0.0);
    //SmartDashboard.putNumber("Shooter/Desired RPM error", Math.abs((desiredRPM - getRPM()) / getRPM()));
    if (Math.abs((desiredRPM - getRPM()) / getRPM()) <= 0.01) {
      SmartDashboard.putBoolean("Shooter/Desired RPM Reached", true);
    }
    else{
      SmartDashboard.putBoolean("Shooter/Desired RPM Reached", false);
    }
    desiredPower = SmartDashboard.getNumber("Shooter/Desired Power", 0.0) ;
    SmartDashboard.putNumber("number I am putting on smartdashbard", desiredRPM);
    //setRPM(desiredRPM);
    SmartDashboard.putNumber("shooter/PID", velocityShooter.withVelocity(desiredRPM/60).Velocity);
    SmartDashboard.getNumber("Shooter/Desired Power?", desiredPower) ;
  }
}
