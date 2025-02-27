package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

    private TalonFX m_infeedMotor;

    private SimpleMotorFeedforward feedforward;
    private double infeedCurrent;
    private double infeedVoltage;
    private double infeedAmperage;

    private double kS = 1;
    private double kV = 0;
    private double kA = 0;

    private double speed;

    public InfeedSS(){
        m_infeedMotor = new TalonFX(InfeedConstants.Infeed_Motor_ID);
        m_infeedMotor.getConfigurator().apply(new TalonFXConfiguration());
        m_infeedMotor.setNeutralMode(NeutralModeValue.Brake);
        m_infeedMotor.setInverted(false);

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
                m_infeedMotor.set(0);
                break;
        
            case SetSpeed:
                m_infeedMotor.set(speed);
                break;

            case CurrentControl:

            case VoltageControl:
                m_infeedMotor.setVoltage(infeedVoltage);
                
            case AmpControl:
                // m_infeedMotor.setcontrol(infeedAmperage);
        }

        SmartDashboard.putNumber("Infeed Motor Speed", speed);
        SmartDashboard.putNumber("Amps", m_infeedMotor.getSupplyCurrent().getValue().in(Amps));
        SmartDashboard.putNumber("Infeed Voltage", m_infeedMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Infeed Constant Volts", m_infeedMotor.getMotorVoltage().getValue().in(Volts));
        SmartDashboard.putNumber("Infeed Velocity", m_infeedMotor.getVelocity().getValue().in(RotationsPerSecond));
    }

    public void Stop(){
        Mode = mode.Stop;
    }

    public void setSpeed(double speed){
        this.speed = speed;
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
