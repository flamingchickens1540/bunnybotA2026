package org.team1540.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

class ArmIOTalonFX implements ArmIO {
    private final TalonFX armTalonFX = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    private final StatusSignal<Current> armStatorCurrent = armTalonFX.getStatorCurrent();
    private final StatusSignal<Current> armSupplyCurrent = armTalonFX.getSupplyCurrent();
    private final StatusSignal<Voltage> armAppliedVoltage = armTalonFX.getMotorVoltage();
    private final StatusSignal<Angle> armPosition = armTalonFX.getPosition();
    private final StatusSignal<Temperature> armMotorTemp = armTalonFX.getDeviceTemp();
    private final StatusSignal<AngularVelocity> armVelocity = armTalonFX.getVelocity();
    private final StatusSignal<Double> armPIDError = armTalonFX.getClosedLoopError();

    private final TalonFXConfiguration armTalonFXconfig = new TalonFXConfiguration();
    private final Slot0Configs armGains = armTalonFXconfig.Slot0;
    private final MotionMagicVoltage armPositionReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut armVoltageReq = new VoltageOut(0);

    private final Debouncer armConnectedDebounce = new Debouncer(0.5);

    ArmIOTalonFX() {

        armGains.kP = ArmConstants.ARM_KP;
        armGains.kI = ArmConstants.ARM_KI;
        armGains.kD = ArmConstants.ARM_KD;
        armGains.kS = ArmConstants.ARM_KS;
        armGains.kV = ArmConstants.ARM_KV;
        armGains.kG = ArmConstants.ARM_KG;
        // set TalonFX cfgs, MotionMagic, slot0

        armTalonFXconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armTalonFXconfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.ARM_SUPPLY_LIMIT_AMPS;
        armTalonFXconfig.CurrentLimits.SupplyCurrentLowerLimit = ArmConstants.ARM_SUPPLY_LOWER_LIMIT_AMPS;
        armTalonFXconfig.CurrentLimits.SupplyCurrentLowerTime = ArmConstants.ARM_SUPPLY_LOWER_TIME_S;

        armTalonFXconfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armTalonFXconfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.ARM_MAX_ANGLE.getRotations();
        armTalonFXconfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armTalonFXconfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ARM_MIN_ANGLE.getRotations();

        armTalonFX.getConfigurator().apply(armTalonFXconfig);

        //        Check if you need this
        MotionMagicConfigs motionMagicConfigs = armTalonFXconfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.ARM_ACCELERATION_RPS2;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ARM_CRUISE_VELOCITY_RPS;

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                armStatorCurrent,
                armSupplyCurrent,
                armAppliedVoltage,
                armPosition,
                armMotorTemp,
                armVelocity,
                armPIDError);
        armTalonFX.optimizeBusUtilization();
    }

    public void setArmPID(double kP, double kI, double kD) {
        Slot0Configs cfgs = armTalonFXconfig.Slot0;
        armTalonFX.getConfigurator().refresh(cfgs);
        cfgs.kP = kP;
        cfgs.kI = kI;
        cfgs.kD = kD;
        armTalonFX.getConfigurator().apply(cfgs);
    }

    public void setArmFF(double kS, double kV, double kG) {
        Slot0Configs cfgs = armTalonFXconfig.Slot0;
        armTalonFX.getConfigurator().refresh(cfgs);
        cfgs.kS = kS;
        cfgs.kV = kV;
        cfgs.kG = kG;
        armTalonFX.getConfigurator().apply(cfgs);
    }

    public void setArmSetpoint(Rotation2d rotations) {
        armPositionReq.withPosition(rotations.getRotations());
    }

    public void setArmVoltage(double voltage) {
        armVoltageReq.withOutput(voltage);
    }

    public void updateInputs(ArmIOInputs inputs) {
        StatusCode armStatus = BaseStatusSignal.refreshAll(
                armStatorCurrent,
                armSupplyCurrent,
                armAppliedVoltage,
                armPosition,
                armMotorTemp,
                armVelocity,
                armPIDError);

        inputs.armConnected = armConnectedDebounce.calculate(armStatus.isOK());

        inputs.armStatorCurrentAmps = armStatorCurrent.getValueAsDouble();
        inputs.armSupplyCurrentAmps = armSupplyCurrent.getValueAsDouble();
        inputs.armMotorAppliedVolts = armAppliedVoltage.getValueAsDouble();
        inputs.armPosition = new Rotation2d(armPosition.getValue());
        inputs.armMotorTemp = armMotorTemp.getValueAsDouble();
        inputs.armMotorVelocityRPS = armVelocity.getValueAsDouble();
        inputs.armPIDError = armPIDError.getValueAsDouble();
    }
}
