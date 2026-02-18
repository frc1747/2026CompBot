// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Controller {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public static final class Kicker {
    public static final int MOTOR_PORT = 4;
    public static final double MOTOR_POWER = 0.2;
  }

  public static final class Drivetrain {
    public static final double MAX_SPEED = 4.1 * 0.5;  // Max speed in m/s  half-speed for now
    public static final double MAX_ACCEL = 4.1;  // Max acceleration in m/s
    public static final double MAX_ANGULAR_VELOCITY = 10.0;  // Rad/s
  }

  public static final class Hood {
    public static final int MOTOR_PORT = 40;
    public static final int ENCODER_PORT_A = 1;
    public static final int ENCODER_PORT_B = 2;
    public static final int COUNTS_PER_REV = 2048;
    public static final double GEAR_RATIO = 17.0;  // 170 tooth rack / 10 tooth pinion
    public static final double TOTAL_HOOD_DEGREES = 19.25;  // 19.25 degrees of hood rotation
    public static final double COUNTS_PER_HOOD_SWEEP = COUNTS_PER_REV * GEAR_RATIO;
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_HOOD_SWEEP / TOTAL_HOOD_DEGREES;

    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double MANUAL_MOTOR_POWER = 0.2;

  }
}
