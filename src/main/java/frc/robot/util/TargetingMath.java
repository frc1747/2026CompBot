package frc.robot.util;

import frc.robot.Constants;

public class TargetingMath {

    public static double getRPMNeededFromDistanceAndAngle(double x, double y) {
        return (Constants.Shooter.SURFACE_A
            + Constants.Shooter.SURFACE_B * x
            + Constants.Shooter.SURFACE_C * y
            + Constants.Shooter.SURFACE_D * Math.pow(x, 2)
            + Constants.Shooter.SURFACE_F * Math.pow(y, 2)
            + Constants.Shooter.SURFACE_E * x * y) / 100;
    }

    public static double getDistanceNeededFromAngleAndRPM(double y, double z) {
        double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_C * y
            + Constants.Shooter.SURFACE_F * Math.pow(y, 2) - z * 100;
        double B = Constants.Shooter.SURFACE_B + Constants.Shooter.SURFACE_E;
        double A = Constants.Shooter.SURFACE_D;
        double positiveRoot = (-B + Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
        if (positiveRoot > 0) return positiveRoot;
        return (-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
    }

    public static double getAngleNeededFromDistanceAndRPM(double x, double z) {
        double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B * x
            + Constants.Shooter.SURFACE_D * Math.pow(x, 2) - z * 100;
        double B = Constants.Shooter.SURFACE_C + Constants.Shooter.SURFACE_E;
        double A = Constants.Shooter.SURFACE_F;
        double positiveRoot = (-B + Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
        if (positiveRoot > 0) return positiveRoot;
        return (-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
    }

    private static double[] searchAngle(double startAngle, double endAngle, double distance) {
        double step = (endAngle >= startAngle) ? 1 : -1;
        for (double angle = startAngle; step > 0 ? angle <= endAngle : angle >= endAngle; angle += step) {
            double rpm = getRPMNeededFromDistanceAndAngle(distance, angle);
            if (rpm <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
                return new double[]{angle, rpm * Constants.Shooter.AUTO_SHOOTER_MULT};
            }
        }
        return null;
    }

    public static double[] findSpeedAndAngleFromDistance(double currentAngle, double distance) {
        double wantedRPM = getRPMNeededFromDistanceAndAngle(distance, currentAngle);
        if (wantedRPM <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
            return new double[]{currentAngle, wantedRPM * Constants.Shooter.AUTO_SHOOTER_MULT};
        }

        double[] result = searchAngle(currentAngle, Constants.Shooter.MAX_HOOD_ANGLE, distance);
        if (result != null) return result;

        result = searchAngle(currentAngle, Constants.Shooter.MIN_HOOD_ANGLE, distance);
        if (result != null) return result;

        return new double[]{-1, -1};
    }
}
