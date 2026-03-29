package frc.robot.util;

import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

public class TargetingMath {

    public static double getRPMNeededFromDistanceAndAngle(double x, double y) {
        return (Constants.Shooter.SURFACE_A
            + Constants.Shooter.SURFACE_B * x
            + Constants.Shooter.SURFACE_C * y
            + Constants.Shooter.SURFACE_D * Math.pow(x, 2)
            + Constants.Shooter.SURFACE_F * Math.pow(y, 2)
            + Constants.Shooter.SURFACE_E * x * y);
    }

    public static double getDistanceNeededFromAngleAndRPM(double y, double z) {
        double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_C * y
            + Constants.Shooter.SURFACE_F * Math.pow(y, 2) - z;
        double B = Constants.Shooter.SURFACE_B + Constants.Shooter.SURFACE_E * y;
        double A = Constants.Shooter.SURFACE_D;
        double positiveRoot = (-B + Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
        if (positiveRoot > 0) return positiveRoot;
        return (-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
    }

    public static double getAngleNeededFromDistanceAndRPM(double x, double z) {
        double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B * x
            + Constants.Shooter.SURFACE_D * Math.pow(x, 2) - z;
        double B = Constants.Shooter.SURFACE_C + Constants.Shooter.SURFACE_E * x;
        double A = Constants.Shooter.SURFACE_F;
        double discriminant = Math.pow(B, 2) - 4 * A * C;
        double rootPlus  = (-B + Math.sqrt(discriminant)) / (2 * A);
        double rootMinus = (-B - Math.sqrt(discriminant)) / (2 * A);
        boolean plusInRange  = rootPlus  >= Constants.Shooter.MIN_HOOD_ANGLE && rootPlus  <= Constants.Shooter.MAX_HOOD_ANGLE;
        boolean minusInRange = rootMinus >= Constants.Shooter.MIN_HOOD_ANGLE && rootMinus <= Constants.Shooter.MAX_HOOD_ANGLE;
        if (minusInRange && !plusInRange) return rootMinus;
        if (plusInRange  && !minusInRange) return rootPlus;
        if (rootMinus > 0) return rootMinus;
        return rootPlus;
    }

    private static double[] searchAngle(double startAngle, double endAngle, double distance) {
        double step = (endAngle >= startAngle) ? 1 : -1;
        for (double angle = startAngle; step > 0 ? angle <= endAngle : angle >= endAngle; angle += step) {
            double rpm = getRPMNeededFromDistanceAndAngle(distance, angle);
            if (rpm <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
                return new double[]{angle, rpm * Constants.Shooter.RPM_CALIBRATION_TRIM};
            }
        }
        return null;
    }

    public static double[] findSpeedAndAngleFromDistance(double currentAngle, double distance) {
        double wantedRPM = getRPMNeededFromDistanceAndAngle(distance, currentAngle);
        if (wantedRPM <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
            return new double[]{currentAngle, wantedRPM * Constants.Shooter.RPM_CALIBRATION_TRIM};
        }

        double[] result = searchAngle(currentAngle, Constants.Shooter.MAX_HOOD_ANGLE, distance);
        if (result != null) return result;

        result = searchAngle(currentAngle, Constants.Shooter.MIN_HOOD_ANGLE, distance);
        if (result != null) return result;

        return new double[]{-1, -1};
    }

    public static List<double[]> getAllValidStatesForDistance(double distance) {
        List<double[]> results = new ArrayList<>();
        for (double angle = Constants.Shooter.MIN_HOOD_ANGLE;
            angle <= Constants.Shooter.MAX_HOOD_ANGLE;
            angle += 1.0) {
            double rpm = getRPMNeededFromDistanceAndAngle(distance, angle);
            if (rpm > 0 && rpm <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
                results.add(new double[]{angle, rpm * Constants.Shooter.RPM_CALIBRATION_TRIM});
            }
        }
        return results;
    }

    public static double[] findPowerOptimizedState(double currentAngle, double distance, double angleDeadband) {
        double optimalAngle = -(Constants.Shooter.SURFACE_C + Constants.Shooter.SURFACE_E * distance)
            / (2 * Constants.Shooter.SURFACE_F);

        optimalAngle = Math.max(Constants.Shooter.MIN_HOOD_ANGLE,
                    Math.min(Constants.Shooter.MAX_HOOD_ANGLE, optimalAngle));

        double targetAngle = (Math.abs(currentAngle - optimalAngle) <= angleDeadband)
            ? currentAngle
            : optimalAngle;

        double rpm = getRPMNeededFromDistanceAndAngle(distance, targetAngle)
            * Constants.Shooter.RPM_CALIBRATION_TRIM;

        return new double[]{targetAngle, rpm};
    }
}
