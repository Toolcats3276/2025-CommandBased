package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfeedConstants;

public class InfeedSS extends SubsystemBase{

    private TalonFX m_infeedMotorLeft;
    private TalonFX m_infeedMotorRight;

    private SimpleMotorFeedforward feedforward;
    private double infeedCurrent;
    private double infeedVoltage;
    private double infeedAmperage;

    private double kS = 1;
    private double kV = 0;
    private double kA = 0;

    private double left_Speed;
    private double right_Speed;

    public InfeedSS(){
        m_infeedMotorLeft = new TalonFX(InfeedConstants.Infeed_Left_Motor_ID);
        m_infeedMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
        m_infeedMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        m_infeedMotorLeft.setInverted(true);

        
        m_infeedMotorRight = new TalonFX(InfeedConstants.Infeed_Right_Motor_ID);
        m_infeedMotorRight.getConfigurator().apply(new TalonFXConfiguration());
        m_infeedMotorRight.setNeutralMode(NeutralModeValue.Brake);
        m_infeedMotorRight.setInverted(false);

        var leftConfigurator = m_infeedMotorLeft.getConfigurator();
        var rightConfigurator = m_infeedMotorRight.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 90;
        limitConfigs.StatorCurrentLimitEnable = true;

        leftConfigurator.apply(limitConfigs);
        rightConfigurator.apply(limitConfigs);



        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        
        
    }

    public enum mode{
        Stop,
        SetSpeed,
        CurrentControl, 
        VoltageControl,
        AmpControl
    }

    mode Mode = mode.Stop;

    @Override

    public void periodic() {

        switch (Mode) {
            
            case Stop:
                m_infeedMotorLeft.set(0);
                break;
        
            case SetSpeed:
                m_infeedMotorLeft.set(left_Speed);
                m_infeedMotorRight.set(right_Speed);
                break;

            case CurrentControl:

            case VoltageControl:
                m_infeedMotorLeft.setVoltage(1);
                m_infeedMotorRight.setVoltage(1);
                
            case AmpControl:
                // m_infeedMotorLeft.setcontrol(infeedAmperage);
        }

        SmartDashboard.putNumber("Infeed Motor Speed", left_Speed);
        SmartDashboard.putNumber("Amps", m_infeedMotorLeft.getSupplyCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("Infeed Voltage", m_infeedMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Infeed Constant Volts", m_infeedMotorLeft.getMotorVoltage().getValue().in(Volts));
        SmartDashboard.putNumber("Infeed Velocity", m_infeedMotorLeft.getVelocity().getValue().in(RotationsPerSecond));
    }

    public void Stop(){
        Mode = mode.Stop;
    }

    public void setSpeed(double left_Speed, double right_Speed){
        this.left_Speed = left_Speed;
        this.right_Speed = right_Speed;
        Mode = mode.SetSpeed;
    }

    public void CurrentControl(double infeedCurrent){
        this.infeedCurrent = infeedCurrent;
        Mode = mode.CurrentControl;
    }

    public void setVoltage(double infeedVoltage){
        this.infeedVoltage = infeedVoltage;
        Mode = mode.VoltageControl;
    }

    public void setAmperage(double infeedAmperage){
        this.infeedAmperage = infeedAmperage;
        Mode = mode.AmpControl;
    }


}
