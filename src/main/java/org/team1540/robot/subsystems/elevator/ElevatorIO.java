package org.team1540.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double rightMotorVelocityMPS;
        public boolean isConnectedRight;
        public double motorTemperatureRightCelsius;
        public double rightStatorCurrentAmps;

        public double leftMotorVelocityMPS;
        public boolean isConnectedLeft;
        public double motorTemperatureLeftCelsius;
        public double leftStatorCurrentAmps;
    }

    public default void moveTo(double position) {}

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void zero() {}

    public default void setVoltage(double volts) {}
}
