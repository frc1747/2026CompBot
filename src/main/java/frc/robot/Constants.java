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

    public static final class Drivetrain {
        public static final double MAX_SPEED = 4.1 * 0.5;  // Max speed in m/s  half-speed for now
        public static final double MAX_ACCEL = 4.1;  // Max acceleration in m/s
        public static final double MAX_ANGULAR_VELOCITY = 10.0;  // Rad/s
    }

    public static final class Intake {
        public static final int MOTOR_PORT = 51;
        public static final double INTAKE_PIVOT_TICK = 6000; //TODO: VERIFY
        public static final double kP = 0.2; //TODO: VERIFY
        public static final double kI = 0.0; //TODO: VERIFY
        public static final double kD = 0.0; //TODO: VERIFY

    }

    public static final class IntakePivot {
        public static final int MOTOR_PORT = 50;
    }

    public static final class Kicker {
        public static final int MOTOR_PORT = 4;
        public static final double MOTOR_POWER = 0.2;
    }
}

