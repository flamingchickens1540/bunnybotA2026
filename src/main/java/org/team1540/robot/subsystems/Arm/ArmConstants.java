package org.team1540.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static Rotation2d ARM_STOW_ANGLE = Rotation2d.fromDegrees(0);
    public static Rotation2d ARM_PLACE_ANGLE = Rotation2d.fromDegrees(0);
    public static Rotation2d ARM_MIN_ANGLE = Rotation2d.fromDegrees(0);
    public static Rotation2d ARM_MAX_ANGLE = Rotation2d.fromDegrees(0);
    public static int ARM_MOTOR_ID = 0;

    public static double ARM_KP = 0;
    public static double ARM_KI = 0;
    public static double ARM_KD = 0;
    public static double ARM_KS = 0;
    public static double ARM_KV = 0;
    public static double ARM_KG = 0;
}