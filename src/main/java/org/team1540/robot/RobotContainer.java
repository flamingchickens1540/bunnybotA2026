package org.team1540.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot.subsystems.elevator.Elevator;
import org.team1540.robot.subsystems.elevator.ElevatorIOTalonFX;

public class RobotContainer {
    private final ElevatorIOTalonFX elevatorIO = new ElevatorIOTalonFX();
    private final Elevator elevator = new Elevator(elevatorIO);
    private final XboxController controller = new XboxController(0);
    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {
        elevator.setDefaultCommand(elevator.setVoltageCommand(() -> controller.getLeftY() * 12));
    }

    private void configureAutoRoutines() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
