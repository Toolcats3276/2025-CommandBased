package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.LEDSS;
import frc.robot.subsystems.SensorSS;

public class LEDDefault extends Command {
  private LEDSS s_Led;
  private SensorSS s_Sensor;
  private ArmSS s_Arm;
  public static Timer blinkTimer;

  public LEDDefault(LEDSS s_Led, SensorSS s_Sensor, ArmSS s_Arm) {
    this.s_Led = s_Led;
    this.s_Sensor = s_Sensor;
    this.s_Arm = s_Arm;
    
    blinkTimer = new Timer();
    addRequirements(s_Led);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if (s_Sensor.algaeSensed()) {
      s_Led.Infeed_Done();
    }

    else if (s_Sensor.coralSensed()) {
      s_Led.Infeed_Done();
    }

    else if (s_Arm.returnSetPoint() == ArmConstants.COMP && s_Sensor.coralSensed()) {
      s_Led.CoralSensed();
    }
    else if (s_Arm.returnSetPoint() == ArmConstants.COMP && s_Sensor.algaeSensed()) {
      s_Led.AlgaeSensed();
    }
    else if (s_Arm.returnSetPoint() == ArmConstants.CORAL_INFEED || s_Arm.returnSetPoint() == ArmConstants.CORAL_SOURCE_INFEED) {
      s_Led.Infeeding();
    }
    else if (s_Arm.returnSetPoint() == ArmConstants.ALGAE_INFEED || 
             s_Arm.returnSetPoint() == ArmConstants.ALGAE_INFEED_L1 || 
             s_Arm.returnSetPoint() == ArmConstants.ALGAE_INFEED_L1_Inverse || 
             s_Arm.returnSetPoint() == ArmConstants.ALGAE_INFEED_L2 || 
             s_Arm.returnSetPoint() == ArmConstants.ALGAE_INFEED_L2_Inverse) {
      s_Led.Infeeding();
    }

    else if (s_Arm.returnSetPoint() == ArmConstants.CLIMB_READY) {
      s_Led.climbing();
    }

    else if (s_Arm.returnSetPoint() == ArmConstants.CLIMBED) {
      s_Led.AlgaeSensed();
    }

    else {
      s_Led.idle();
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
