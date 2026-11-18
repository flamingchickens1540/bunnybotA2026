package org.team1540.robot.subsystems.Drivetrain;

import static org.team1540.robot.subsystems.Drivetrain.DrivetrainConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Objects;
import java.util.Queue;
import org.team1540.robot.subsystems.Drivetrain.frc.robot.generated.TunerConstants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, CAN_BUS);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon2() {
        pigeon.getConfigurator()
                .apply(Objects.requireNonNullElse(
                        TunerConstants.DrivetrainConstants.Pigeon2Configs, new Pigeon2Configuration()));
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }
}
