package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;

public class CoralInfeedSensorCommand extends SequentialCommandGroup{



    public CoralInfeedSensorCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(  
                new ConditionalCommand(
                    //while true
                    new ParallelCommandGroup(
                        new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT),
                        new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT),
                        new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                        new InfeedCommand(s_Infeed, 0),
                        new SequentialCommandGroup(
                            new WaitCommand(0.1),
                            new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(true))
                        )
                    ),
                    //while false
                    new ParallelCommandGroup(
                        new ConditionalCommand(
                            new ParallelCommandGroup(                        
                                new ArmPIDCommand(s_Arm, ArmConstants.CORAL_INFEED, ArmConstants.MAX_PID_OUTPUT),
                                new WristPIDCommand(s_Wrist, WristConstants.CORAL_INFEED, WristConstants.MAX_PID_OUTPUT),
                                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                                new InfeedCommand(s_Infeed, InfeedConstants.INFEED)
                            ),
                            new ParallelCommandGroup(
                                new WristPIDCommand(s_Wrist, WristConstants.CORAL_INFEED, WristConstants.MAX_PID_OUTPUT),
                                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT),
                                new InfeedCommand(s_Infeed, InfeedConstants.INFEED)
                            ),
                            () -> s_Elevator.atAutoCompPose())
        
                    ),

                    //condition
                    () -> s_Sensor.coralSensed()
                )
            )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}