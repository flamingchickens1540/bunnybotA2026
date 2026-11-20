package org.team1540.robot.subsystems.elevator;

import static org.team1540.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.util.LoggedTracer;

public class Elevator extends SubsystemBase {

    private final ElevatorIO IO;
    private final ElevatorIOInputsAutoLogged ioInputs = new ElevatorIOInputsAutoLogged();

    enum ElevatorState {
        STOW(0),
        SCORING(maxHeightMeters),
        PICKUP(pickUpHeightMeters); // in meters
        public final double position;

        ElevatorState(double position) {
            this.position = position;
        }
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.IO = elevatorIO;
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, minHeightMeters, maxHeightMeters);
        setpoint = positionMeters;
    }

    public boolean isAtSetpoint(double positionTolerance) {
        return MathUtil.isNear(setpoint, positionMeters, positionTolerance);
    }

    public void setVoltage(double volts) {
        IO.setVoltage(volts);
    }

    public Command setVoltageCommand(DoubleSupplier volts) {
        return Commands.runOnce(
                () -> {
                    double voltsAsDouble = volts.getAsDouble();
                    setVoltage(voltsAsDouble);
                },
                this);
    }

    public Command zero() {
        return Commands.runOnce(() -> {
            IO.zero();
        });
    }

    public Command goToSetpoint(ElevatorState elevatorState) {
        return Commands.runOnce(() -> {
                    IO.moveTo(elevatorState.position);
                    setPosition(elevatorState.position);
                })
                .until(() -> isAtSetpoint(positionToleranceMeters));
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();
        IO.updateInputs(ioInputs);
        Logger.processInputs("Elevator", ioInputs);
        LoggedTracer.record("Elevator");
    }

    public static Elevator createReal() {
        return new Elevator(new ElevatorIOTalonFX());
    }
}
