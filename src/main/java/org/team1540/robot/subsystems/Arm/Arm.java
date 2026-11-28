package org.team1540.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.subsystems.Arm.ArmConstants.*;
import org.team1540.robot.util.LoggedTracer;
import org.team1540.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private static boolean hasInstance = false;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private Rotation2d setpoint = new Rotation2d();

    private final Alert armDisconnectedAlert = new Alert("Arm disconnected", Alert.AlertType.kError);

    // States of the arm
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

    // Trapezoid profile of pid
    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ArmConstants.ARM_CRUISE_VELOCITY_RPS, ArmConstants.ARM_ACCELERATION_RPS2));

    // Constructor
    private Arm(ArmIO armIO) {
        if (hasInstance) throw new IllegalStateException("Instance already exists");
        hasInstance = true;
        io = armIO;
    }

    @Override
    public void periodic() {

        // Processing data
        LoggedTracer.reset();
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Updates PID/FF of motor if changed during tuning
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setArmFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setArmPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

        armDisconnectedAlert.set(!inputs.armConnected);

        // Logging data
        LoggedTracer.record("Arm");
    }

    public void holdPosition() {
        setPosition(getPosition());
    }

    // moves arm to position
    public void setPosition(Rotation2d position) {
        setpoint = Rotation2d.fromRotations(MathUtil.clamp(
                position.getRotations(),
                ArmConstants.ARM_MIN_ANGLE.getRotations(),
                ArmConstants.ARM_MAX_ANGLE.getRotations()));
        io.setArmSetpoint(setpoint);
    }

    // returns arm position
    public Rotation2d getPosition() {
        //        return inputs.armPosition;
        return new Rotation2d();
    }

    // changes brake mode
    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    // returns if arm is at setpoint
    @AutoLogOutput(key = "Arm/AtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpoint.getRotations(), getPosition().getRotations(), ArmConstants.ARM_ERROR_TOLERANCE);
    }

    // returns current setpoint
    @AutoLogOutput(key = "Arm/Setpoint")
    public Rotation2d getSetpoint() {
        return setpoint;
    }

    // returns a command for moving arm to a setpoint
    public Command commandToSetpoint(Rotation2d position) {
        return Commands.run(() -> setPosition(position), this).until(this::isAtSetpoint);
    }

    // returns a command for moving arm to an arm state
    public Command commandToSetpoint(ArmState state) {
        return commandToSetpoint(state.position());
    }

    // returns time to setpoint
    @AutoLogOutput(key = "Arm/TimeToSetpoint")
    public double timeToSetpoint() {
        return isAtSetpoint() ? 0 : timeToSetpoint(getSetpoint());
    }

    // helper
    public double timeToSetpoint(Rotation2d setpoint) {
        trapezoidProfile.calculate(
                0,
                new TrapezoidProfile.State(getPosition().getRotations(), inputs.armMotorVelocityRPS),
                new TrapezoidProfile.State(setpoint.getRotations(), 0));
        return trapezoidProfile.totalTime();
    }

    // returns a real arm
    public static Arm createReal() {
        return new Arm(new ArmIOTalonFX());
    }

    public static Arm createDummy() {
        return new Arm(new ArmIO() {});
    }

    public static Arm createSim() {
        return new Arm(new ArmIO() {});
    }
}
