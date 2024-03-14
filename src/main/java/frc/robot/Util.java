// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Util {

    // Use Desmos Exponential Regression Calculator to get constants
    public static double getArmAngleForShooter(double distance) {
        final double a = 3.0;
        final double b = 1.0;

        return (a * Math.pow(b, distance));
    }

    public static boolean inRange(double value, double min, double max) {
        return (value <= max) && (value >= min);
    }

    public static double map(double Value, double FromMin, double FromMax, double ToMin, double ToMax) {
        return (((Value - FromMin) / (FromMax - FromMin)) * (ToMax - ToMin)) + ToMin;
    }
}
