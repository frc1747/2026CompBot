// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Vision {
    // Local hostnames of the unique Limelights on the system
    // WARNING: IF YOU CHANGE OUT THE HARDWARE, ENSURE TO PROPERLY
    // SET THE HOSTNAME ON THE LIMELIGHT TO COORESPOND WITH ITS 
    // LOCATION ON THE BOT!!! 
    public static final String LIMELIGHT_LEFT = "limelight-left";
    public static final String LIMELIGHT_RIGHT = "limelight-right";
    public static final String LIMELIGHT_TURRET = "limelight-turret";

    // List of the active Limelights on the system to be used for Pose2D estimation
    // Add any Limelights defined above to this list.
    public static final List<String> ACTIVE_POSE_LIMELIGHTS = List.of(
      LIMELIGHT_LEFT,
      LIMELIGHT_RIGHT
    );

    // VISION_STDDEVS allows us to control how much we trust the values coming from the Limelight(s).
    // The higher the value (distance standard deviations), the less we trust it.
    // 
    // n1: X Position Standard Deviations in meters
    //     How wrong do we think vision could be about where we am on the field in X?
    // n2: Y Position Standard Deviations in meters
    //     How wrong do we think vision could be about where we are on the field in Y?
    // n3: Rotation (theta) Standard Deviations in RADIANS
    //     How wrong is vision about our heading?
    //
    // 0.7, 0.7, and 9999999 tells the code that we are somewhat trusting distant april tags
    // and basically completely trusting the Pigeon for Yaw.
    public static final Matrix<N3, N1> VISION_STDDEVS = VecBuilder.fill(0.7, 0.7, 9999999);

    public static final double FIELD_CENTER_X = 8.7741252;
    public static final double FIELD_CENTER_Y = 4.0259508;
  }

  public static final class Controller {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public static final class Drivetrain {
    public static final double MAX_SPEED = 4.1 * 0.5;  // Max speed in m/s  half-speed for now
    public static final double MAX_ACCEL = 4.1;  // Max acceleration in m/s
    public static final double MAX_ANGULAR_VELOCITY = 10.0;  // Rad/s
  }
  public static final class Hopper {
    public static final int MOTOR_PORT = 44;
    public static final double MOTOR_POWER = 0.2;
  }

  public static final class Hood {
    public static final int MOTOR_PORT = 40;
    public static final int ENCODER_PORT_A = 5;
    public static final int ENCODER_PORT_B = 6;
    public static final int COUNTS_PER_REV = 2048;
    public static final double GEAR_RATIO = 17.0;  // 170 tooth rack / 10 tooth pinion
    public static final double TOTAL_HOOD_DEGREES = 19.25;  // 19.25 degrees of hood rotation
    public static final double COUNTS_PER_HOOD_SWEEP = COUNTS_PER_REV * GEAR_RATIO;
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_HOOD_SWEEP / TOTAL_HOOD_DEGREES;
    public static final double STARTING_ANGLE = 25.0;  // Angle that the hood starts at

    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double MANUAL_MOTOR_POWER = 0.2;

  }

  public static final class Turret {
    public static final int MOTOR_PORT = 59;
    public static final int ENCODER_PORT_A = 0;
    public static final int ENCODER_PORT_B = 1;
    // left from the perspective of someone facing towards the turret side of bot
    public static final int LEFT_LIMIT_PORT = 2; 
    public static final int RIGHT_LIMIT_PORT = 3;
    // gear ratio of 396 to 1 here probably but needs to be tested
    //public static final double TURRETRATIO = 7.33333333333333333333333; 
    public static final double TURRET_RATIO = 166.9;
    public static final int encoderLimit = 5771 / 2; // temporary encoder value limit
    public static final double PID_P = 0;
    public static final double PID_I = 0; // needs tuning
    public static final double PID_D = 0;
    public static final double GO_TO_ANGLE_LOWER_SAFETY = -1;
    public static final double GO_TO_ANGLE_HIGHER_SAFETY = 1;
    public static final double UPPER_LIMIT = 90;
    public static final double LOWER_LIMIT = -90;
    public static final double DIST_TO_BOT_CENTER = 0.1529842; // meters
  }

  public static final class Shooter {
    public static final int MOTOR_LEFT_PORT = 41;
    public static final int MOTOR_RIGHT_PORT = 42;
    public static final int MOTOR_HOOD_PORT = 40;
    public static final int ENCODER_PORT = 1; // needs to be set
    public static final double ENCODER_OFFSET = .2; // needs to be set o7
    public static final double SURFACE_A = 44.1596; // needs tuning
    public static final double SURFACE_B = -4.3595; // needs tuning
    public static final double SURFACE_C = -0.94726; // needs tuning
    public static final double SURFACE_D = 0.12545; // needs tuning
    public static final double SURFACE_E = 0.36687; // needs tuning
    public static final double SURFACE_F = 0.017314; // needs tuning
    public static final double MAX_AUTOSHOOT_POWER =.85;
    public static final double MAX_HOOD_ANGLE = 43;// degrees
    public static final double MIN_HOOD_ANGLE = 26;// degrees
    public static final double PID_P = 0;
    public static final double PID_I = 0;
    public static final double PID_D = 0;
  }
     public static final class Intake {
        public static final int MOTOR_PORT = 46;
        public static final double POWER = 0.2;
        public static final int INTAKE_PIVOT_TICK = 6000; //TODO: VERIFY
        public static final double kP = 0.2; //TODO: VERIFY
        public static final double kI = 0.0; //TODO: VERIFY
        public static final double kD = 0.0; //TODO: VERIFY

    }

    public static final class IntakePivot {
        public static final int MOTOR_PORT = 45;
    }

    public static final class Kicker {
        public static final int MOTOR_PORT = 43;
        public static final double MOTOR_POWER = 0.35;
    }
}


