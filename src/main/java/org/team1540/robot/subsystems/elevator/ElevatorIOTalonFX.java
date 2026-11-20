package org.team1540.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorCANID);
    private final TalonFX leftMotor = new TalonFX(ElevatorConstants.leftMotorCANID);

    private final StatusSignal<Temperature> temperatureRight = rightMotor.getDeviceTemp();
    private final StatusSignal<Current> currentRight = rightMotor.getSupplyCurrent();
    private final StatusSignal<Current> currentStatorRight = rightMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> rightMotorVelocity = rightMotor.getVelocity();

    private final StatusSignal<Temperature> temperatureLeft = leftMotor.getDeviceTemp();
    private final StatusSignal<Current> currentLeft = leftMotor.getSupplyCurrent();
    private final StatusSignal<Current> currentStatorLeft = leftMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> leftMotorVelocity = leftMotor.getVelocity(); // returns rotations/second
    private final Follower follower = new Follower(ElevatorConstants.rightMotorCANID, MotorAlignmentValue.Opposed);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ElevatorIOTalonFX() {
        leftMotor.setControl(follower);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.Feedback.SensorToMechanismRatio = ElevatorConstants.gearRatio;
        configs.CurrentLimits.SupplyCurrentLimit = 67;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLowerLimit = 40;
        configs.CurrentLimits.SupplyCurrentLowerTime = 1.5;
        configs.Slot0.kP = ElevatorConstants.kp;
        configs.Slot0.kI = ElevatorConstants.ki;
        configs.Slot0.kD = ElevatorConstants.kd;
        configs.Slot0.kG = ElevatorConstants.kg;
        configs.Slot0.kV = ElevatorConstants.kv;
        configs.Slot0.kS = ElevatorConstants.ks; // static friction
        configs.Slot0.kA = ElevatorConstants.ka;
        configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.cruiseConstant;
        configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.magicAcceleration;
        rightMotor.getConfigurator().apply(configs);
        leftMotor.getConfigurator().apply(configs);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isConnectedLeft = BaseStatusSignal.refreshAll(
                        temperatureLeft, currentLeft, currentStatorLeft, leftMotorVelocity)
                .isOK();
        inputs.isConnectedRight = BaseStatusSignal.refreshAll(
                        temperatureRight, currentRight, currentStatorRight, rightMotorVelocity)
                .isOK();

        inputs.leftStatorCurrentAmps = currentStatorLeft.getValueAsDouble();
        inputs.rightStatorCurrentAmps = currentStatorRight.getValueAsDouble();

        inputs.motorTemperatureLeftCelsius = temperatureLeft.getValueAsDouble();
        inputs.motorTemperatureRightCelsius = temperatureRight.getValueAsDouble();

        inputs.leftMotorVelocityMPS = leftMotorVelocity.getValueAsDouble();
        inputs.rightMotorVelocityMPS = rightMotorVelocity.getValueAsDouble();
    }

    @Override
    public void moveTo(double position) {
        rightMotor.setControl(positionControl.withPosition(position));
        leftMotor.setControl(follower);
    }

    @Override
    public void zero() {
        rightMotor.setPosition(0);
        leftMotor.setPosition(0);
    }

    @Override
    public void setVoltage(double volts) {
        rightMotor.setControl(voltageControl.withOutput(volts));
        leftMotor.setControl(follower);
    }
}
