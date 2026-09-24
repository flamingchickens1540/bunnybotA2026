package frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class ExampleIOTalonFX implements ExampleIO{
    private final TalonFX exampleMotor = new TalonFX(ExampleSubsystemConstants.EXAMPLE_MOTOR_ID);

    private final StatusSignal<Current> exampleMotorStatorCurrentAmps = exampleMotor.getStatorCurrent();
    private final StatusSignal<Current> exampleMotorSupplyCurrentAmps = exampleMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> exampleMotorTempCelsius = exampleMotor.getDeviceTemp();

    public ExampleIOTalonFX(){
        TalonFXConfiguration exampleMotorTalonFXConfig = new TalonFXConfiguration();
        exampleMotorTalonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        exampleMotorTalonFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        exampleMotorTalonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        exampleMotorTalonFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        exampleMotorTalonFXConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive;
        exampleMotorTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        exampleMotor.getConfigurator().apply(exampleMotorTalonFXConfig);
    }

    @Override
    public void updateInputs(ExampleInputs inputs) {
        inputs.exampleMotorConnected = exampleMotor.isConnected();
        inputs.exampleMotorStatorCurrentAmps = exampleMotorStatorCurrentAmps.getValueAsDouble();
        inputs.exampleMotorSupplyCurrentAmps = exampleMotorSupplyCurrentAmps.getValueAsDouble();
        inputs.exampleMotorTempCelsius = exampleMotorTempCelsius.getValueAsDouble();
    }

    @Override
    public void setExampleMotorVoltage(double voltage) {
        exampleMotor.setVoltage(voltage);
    }
}
