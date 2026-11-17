package org.team1540.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.team1540.robot.subsystems.Arm.ArmConstants.*;
import org.team1540.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private static boolean hasInstance = false;

    private final ArmIO io;
    //    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private Rotation2d setpoint = new Rotation2d();

    //    Alerts n things

    public enum ArmState {
        STOW(new LoggedTunableNumber("Arm/Setpoints/Stow", 0)),
        INTAKE_STANDING(new LoggedTunableNumber("Arm/Setpoints/IntakeStandingDegrees", 180)),
        INTAKE_TIPPED(new LoggedTunableNumber("Arm/Setpoints/IntakeTippedDegrees", 160));

        private final DoubleSupplier positionDegrees;

        ArmState(DoubleSupplier positionDegrees) {
            this.positionDegrees = positionDegrees;
        }

        public Rotation2d position() {
            return Rotation2d.fromDegrees(positionDegrees.getAsDouble());
        }
    }

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", ArmConstants.ARM_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", ArmConstants.ARM_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", ArmConstants.ARM_KD);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", ArmConstants.ARM_KG);
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", ArmConstants.ARM_KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", ArmConstants.ARM_KV);

    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ArmConstants.ARM_CRUISE_VELOCITY_RPS, ArmConstants.ARM_ACCELERATION_RPS2));

    private Arm(ArmIO armIO) {
        if (hasInstance) throw new IllegalStateException("Instance already exists");
        hasInstance = true;
        io = armIO;
    }

    public void periodic() {}

    public void holdPosition() {}

    public void setPosition(Rotation2d position) {}

    public Rotation2d getPosition() {}
}
