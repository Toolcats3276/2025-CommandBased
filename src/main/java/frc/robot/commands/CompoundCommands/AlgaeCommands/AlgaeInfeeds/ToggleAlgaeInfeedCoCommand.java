// package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
// import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.ElevatorSS;
// import frc.robot.subsystems.InfeedSS;
// import frc.robot.subsystems.SensorSS;
// import frc.robot.subsystems.WristSS;

// public class ToggleCoralInfeedCoCommand extends SequentialCommandGroup{



//     public ToggleCoralInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor, BooleanSupplier endRepeatCommand) {

//         addCommands(
//             new ConditionalCommand(
//                 new CoralSourceInfeedSensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
//                     .until(endRepeatCommand),

//                 new CoralInfeedSensorCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
//                     .until(endRepeatCommand),
                
//                 //condition
//                 () -> s_Arm.returnSetPoint() == ArmConstants.CORAL_INFEED
//             )
//         );
//         addRequirements(s_Infeed, s_Wrist, s_Arm, s_Elevator);
//     }
    
   
// }
package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InfeedConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.CompoundCommands.CompCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1CoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL2CoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class ToggleAlgaeInfeedCoCommand extends SequentialCommandGroup{



    public ToggleAlgaeInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor, BooleanSupplier endRepeatCommand) {

        addCommands(
            new RepeatCommand(
                
                new ConditionalCommand(
                    //on true
                        new ConditionalCommand(
                            //on true
                                new ConditionalCommand(
                                    //on true
                                        new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                new WristPIDCommand(s_Wrist, WristConstants.ALGAE_INFEED_L1_COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                                                new SequentialCommandGroup(
                                                    new WaitCommand(0.5),
                                                    new ParallelCommandGroup(
                                                        new ArmPIDCommand(s_Arm, ArmConstants.ALGAE_INFEED_L1, ArmConstants.MAX_PID_OUTPUT),
                                                        new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                                                    ),
                                                    new WaitCommand(0.5),
                                                    new InstantCommand(() -> s_Infeed.setVoltage(1))
                                                )
                                            )
                                        ),        
                                    //onFalse
                                        new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                new WristPIDCommand(s_Wrist, WristConstants.ALGAE_INFEED_L1_COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                                                new SequentialCommandGroup(
                                                    new WaitCommand(0.5),
                                                    new ParallelCommandGroup(
                                                        new ArmPIDCommand(s_Arm, ArmConstants.ALGAE_INFEED_L1, ArmConstants.MAX_PID_OUTPUT),
                                                        new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                                                    ),
                                                    new WaitCommand(0.5),
                                                    new InstantCommand(() -> s_Infeed.setVoltage(1))
                                                )
                                            )
                                        ),  

                                    //condition
                                    () -> s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L2
                                ),
                            //on false
                                new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                            new WaitCommand(0.2),
                                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                                        ),
                                        new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT),
                                        new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                                        // new InfeedCommand(s_Infeed,0.5)
                                        new InstantCommand(() -> s_Infeed.setVoltage(1))
                                    )
                            ),

                            () -> s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L1 ||
                                  s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L2
                        ),
                    //on false

                        new ConditionalCommand(    
                            //on true
                                new ConditionalCommand(
                                    //onTrue
                                        new ParallelCommandGroup(
                                            new AlgaeInfeedL2CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                            new PrintCommand("L2 Infeed")
                                        ),
        
                                    //onFalse
                                        new ParallelCommandGroup(
                                            new AlgaeInfeedL1CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                            new PrintCommand("L1 Infeed")
                                        ),
        
                                    //Condition
                                        () -> s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L2
                                ),

                            //on false
                                new ParallelCommandGroup(
                                    new AlgaeInfeedCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                    new PrintCommand("Ground Infeed")
                                ),

                            //condition
                                () -> s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L1 ||
                                s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L2
                        ),
                    //condition
                    () -> s_Sensor.algaeSensed()
                )
            )
            .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND)))
        );
        addRequirements(s_Infeed, s_Wrist, s_Arm, s_Elevator);
    }
    
   
}