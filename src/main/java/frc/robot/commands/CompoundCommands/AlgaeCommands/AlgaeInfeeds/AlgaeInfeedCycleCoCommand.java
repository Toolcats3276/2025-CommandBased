// package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

// import java.util.Map;

// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.wpilibj2.command.SelectCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.SensorSS;

// public class AlgaeInfeedCycleCoCommand extends SequentialCommandGroup{
    
//     private boolean endCommand = false;

//     public AlgaeInfeedCycleCoCommand(SensorSS s_Sensor){

//         addCommands(
//             new RepeatCommand(
//                 new ConditionalCommand(
//                     //comp commands
//                     new ParallelCommandGroup(
//                         new SelectCommand<>(
//                             Map.ofEntries(
//                                 Map.entry(SensorSS.AlgaeInfeedState.FLOOR, new PrintCommand("Floor Comp")),
//                                 Map.entry(SensorSS.AlgaeInfeedState.L2, new PrintCommand("L2 Comp")),
//                                 Map.entry(SensorSS.AlgaeInfeedState.L3, new PrintCommand("L3 Comp"))
//                             ), 
//                             s_Sensor::getAlgaeInfeedState
//                         ),
//                         new InstantCommand(() -> endCommand = true)
//                     ), 
//                     //infeed commands
//                     new ParallelCommandGroup(
//                         new SelectCommand<>(
//                             Map.ofEntries(
//                                 Map.entry(SensorSS.AlgaeInfeedState.FLOOR, new PrintCommand("Floor Comp")),
//                                 Map.entry(SensorSS.AlgaeInfeedState.L2, new PrintCommand("L2 Comp")),
//                                 Map.entry(SensorSS.AlgaeInfeedState.L3, new PrintCommand("L3 Comp"))
//                             ), 
//                             s_Sensor::getAlgaeInfeedState
//                         ),
//                         new InstantCommand(() -> endCommand = false)
//                     ),
//                     //condition
//                     () -> s_Sensor.algaeSensed()
//                 )
//             ).until(() -> endCommand).finallyDo(s_Sensor::resetAlgaeInfeedState)
//         );
//     }
// }
package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SensorSS;

public class AlgaeInfeedCycleCoCommand extends SequentialCommandGroup{

    private boolean endCommand;

    /**
     * Command to cycle through infeed commands with one button and reset to a known starting state/infeed command. 
     * Each infeed can have a unique compliance routine.
     * This can be tiled for any number of unique commands/states.
     * @param cycleStates Subsystem with cycleable enum state chooser. See SensorSS.java for subsystem requirements.
     * @param sensor Sensor input, should be from a sensor subsystem not a boolean supplier.
     */
    public AlgaeInfeedCycleCoCommand(SensorSS s_Sensor, BooleanSupplier sensor){

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    //Compliance selectable commands.
                    new ParallelCommandGroup(
                        new InstantCommand(() -> endCommand = true),
                        new SelectCommand<>(
                            Map.ofEntries(
                                Map.entry(SensorSS.AlgaeInfeedState.FLOOR, new PrintCommand("Floor Comp")),
                                Map.entry(SensorSS.AlgaeInfeedState.L2, new PrintCommand("L2 Comp")),
                                Map.entry(SensorSS.AlgaeInfeedState.L3, new PrintCommand("L3 Comp"))
                            ),
                            s_Sensor::getAlgaeInfeedState)
                    ),

                    //Infeed selectable commands.
                    new ParallelCommandGroup(
                        new InstantCommand(() -> endCommand = false),
                        new SelectCommand<>(
                            Map.ofEntries(
                                Map.entry(SensorSS.AlgaeInfeedState.FLOOR, new PrintCommand("Floor Infeed")),
                                Map.entry(SensorSS.AlgaeInfeedState.L2, new PrintCommand("L2 Infeed")),
                                Map.entry(SensorSS.AlgaeInfeedState.L3, new PrintCommand("L3 Infeed"))
                            ),
                            s_Sensor::getAlgaeInfeedState)
                    ),

                    //Sensor input.
                    sensor
                )
            ).until(() -> endCommand).finallyDo(s_Sensor::resetAlgaeInfeedState)
        );
    } 
}