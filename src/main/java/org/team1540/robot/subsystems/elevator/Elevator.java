package org.team1540.robot.subsystems.elevator;

import static org.team1540.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.util.LoggedTracer;

public class Elevator extends SubsystemBase {

    private final TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(0, 1); // TODO:set max velocity and max acceleration
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

    private Elevator(ElevatorIO elevatorIO) {
        this.IO = elevatorIO;
    }

    public void setPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, 0.0, rotationsMaxHeightMeters);
        setpoint = positionMeters;
    }

    public boolean isAtSetpoint(double positionTolerance) {
        return MathUtil.isNear(setpoint, position, positionTolerance, 0.0, rotationsMaxHeightMeters);
    }

    public void setVoltage(double volts) {
        IO.setVoltage(volts);
    }

    public Command setVoltageCommand(double volts) {
        return Commands.runOnce(() -> {
            setVoltage(volts);
        });
    }

    public Command zero() {
        return Commands.runOnce(() -> {
            IO.zero();
        });
    }

    public Command goToSetpoint(ElevatorState elevatorState) {
        return Commands.run(() -> {
                    IO.moveTo(elevatorState.position);
                })
                .until(() -> isAtSetpoint(positionToleranceMeters));
    }

    @Override
    public void periodic() {
        LoggedTracer.reset();
        IO.updateInputs(ioInputs);
        Logger.processInputs("Elevator",ioInputs);
        LoggedTracer.record("Elevator");
    }
    public static Elevator createReal(){
        return new Elevator(new ElevatorIOTalonFX());
    }
}
