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
package frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.commands.CompoundCommands.CompCoCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.InfeedSS;
import frc.robot.subsystems.SensorSS;
import frc.robot.subsystems.WristSS;

public class ToggleCoralInfeedCoCommand extends SequentialCommandGroup{



    public ToggleCoralInfeedCoCommand(ArmSS s_Arm, InfeedSS s_Infeed, WristSS s_Wrist, ElevatorSS s_Elevator, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(
                
                new ConditionalCommand(
                    //on true
                        new ConditionalCommand(
                            //on true
                            new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                    new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.MAX_PID_OUTPUT),
                                    new InfeedCommand(s_Infeed, 0, 0)
                                ),
                                new WaitCommand(0.25),
                                new ParallelCommandGroup(
                                    new ArmPIDCommand(s_Arm, ArmConstants.COMP, ArmConstants.MAX_PID_OUTPUT),
                                    new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                                ),
                                new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(true))
                            ),
                            //on false
                            new SequentialCommandGroup(
                                new InfeedCommand(s_Infeed, 0, 0),
                                new InfeedCommand(s_Infeed, -0.23, -0.23),
                                new WaitCommand(0.2),
                                new InfeedCommand(s_Infeed, 0, 0),
                                new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(true))
                            ),
                            () -> s_Sensor.getInfeedState()),

                    //on false
                        new ParallelCommandGroup(
                            new InstantCommand(() -> s_Sensor.setEndCoralInfeedCommand(false)),
                            new ConditionalCommand(
                                //on true
                                new CoralInfeedCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor),
                                //on false
                                new CoralSourceInfeedCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed),
                                //condition
                                () -> s_Sensor.getInfeedState()
                            )
                        ),
                    //condition
                    () -> s_Sensor.coralSensed()
                    
                )
            )
            .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setInfeedState(false)))
        );
        addRequirements(s_Infeed, s_Wrist, s_Arm, s_Elevator);
    }
    
   
}