package org.team1540.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final int rightMotorCANID = 16;
    public static final int leftMotorCANID = 15;

    public static final double gearRatio = 3.0;
    public static final double drumRadius = 47.75 / 2000.0;
    // set value later

    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = 1.0414;
    public static double pickUpHeightMeters;
    public static final double maxHeightRevolutions = maxHeightMeters / (drumRadius * 2 * StrictMath.PI);

    public static double positionMeters; // in meters
    public static final double motorCircumference = 0;
    public static double kp; // proportional
    public static double kd; // integral
    public static double ki; // derivative
    public static double kg; // gravity
    public static double kv; // velocity
    public static double ks; // Static feedforward gain.
    public static double ka; // acceleration
    public static double cruiseConstant = 5;
    public static double magicAcceleration = 2500;
    public static double setpoint;
    public static double positionToleranceMeters = 1 / 100.0;
}
