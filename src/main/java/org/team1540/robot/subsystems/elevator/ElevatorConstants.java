package org.team1540.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorConstants {
    public static final int rightMotorCANID = 0;
    public static final int leftMotorCANID = 0;
    public static final TalonFX rightMotor = new TalonFX(0);
    public static double rightMotorVelocity = rightMotor.getVelocity().getValueAsDouble();

    public static final double gearRatio = 0; // set value later

    public static final TalonFX leftMotor = new TalonFX(0);
    public static double leftMotorVelocity = leftMotor.getVelocity().getValueAsDouble();

    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = 1.0414; // might need to change min and max height

    public static double position; // 1 = 360 degrees
    public static final double motorCircumference = 0;
    public static double kp; // proportional
    public static double kd; // integral
    public static double ki; // derivative
    public static double kg; // gravity
    public static double kv; // velocity
    public static double ks; // Static feedforward gain.
    public static double ka; // acceleration
    public static double cruiseConstant;
    public static double magicAcceleration;
    public static double setpoint;
    public static double rotationsMaxHeightMeters;
    public static double pickUpHeightMeters; // TODO: SET LATER
    public static double positionToleranceMeters = 1 / 100.0;
}
