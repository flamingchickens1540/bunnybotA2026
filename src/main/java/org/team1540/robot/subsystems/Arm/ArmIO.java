package org.team1540.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class IntakeInputs {
        public boolean armConnected;
        public Rotation2d armPosition;
        public double armMotorVelocityRPS;
        public double armMotorAppliedVolts;
        public double armSupplyCurrentAmps;
        public double armStatorCurrentAmps;
        public double armMotorTemp;
    }

    default void setArmPID(double kP, double kI, double kD) {}

    default void setArmFF(double kS, double kV, double kG) {}

    default void setArmSetpoint(Rotation2d rotations) {}

    default void resetArmPosition(Rotation2d rotations) {}

    default void setArmVoltage(double voltage) {}

    default void updateInputs(IntakeInputs inputs) {}
}