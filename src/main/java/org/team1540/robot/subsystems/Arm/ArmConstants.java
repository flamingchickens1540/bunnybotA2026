package org.team1540.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(0);
    public static Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(0);
    public static int ARM_MOTOR_ID = 67;

    public static double ARM_KP = 0;
    public static double ARM_KI = 0;
    public static double ARM_KD = 0;
    public static double ARM_KS = 0;
    public static double ARM_KV = 0;
    public static double ARM_KG = 0;

    public static double ARM_SUPPLY_LIMIT_AMPS = 50;
    public static double ARM_SUPPLY_LOWER_LIMIT_AMPS = 0.1;
    public static double ARM_SUPPLY_LOWER_TIME_S = 15;
    public static double ARM_GEAR_RATIO = 1.0;

    public static double ARM_CRUISE_VELOCITY_RPS = 0;
    public static double ARM_ACCELERATION_RPS2 = 0;

    public static double ARM_ERROR_TOLERANCE = 0.01;
}
