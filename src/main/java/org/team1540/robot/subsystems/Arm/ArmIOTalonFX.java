package org.team1540.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;


class ArmIOTalonFX implements ArmIO {
    private final TalonFX armTalonFX = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    ArmIOTalonFX() {
        TalonFXConfiguration armTalonFXconfig = new TalonFXConfiguration();
    
        Slot0Configs armGains = armTalonFXconfig.Slot0;
        armGains.kP = ArmConstants.ARM_KP;
        armGains.kI = ArmConstants.ARM_KI;
        armGains.kD = ArmConstants.ARM_KD;
        armGains.kS = ArmConstants.ARM_KS;
        armGains.kV = ArmConstants.ARM_KV;
        armGains.kG = ArmConstants.ARM_KG;
        // set TalonFX cfgs, MotionMagic, slot0
    }

    public void setArmPID(double kP, double kI, double kD) {

    }

    public void setArmFF(double kS, double kV, double kG) {

    }

    public void setArmSetpoint(Rotation2d rotations) {

    }

    public void resetArmPosition(Rotation2d rotations) {

    }

    public void setArmVoltage(double voltage) {

    }

    public void updateInputs(IntakeInputs inputs) {

    }
}