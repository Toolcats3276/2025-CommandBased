// package frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L1CoCommands;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.WristConstants;
// import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
// import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
// import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
// import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
// import frc.robot.subsystems.ArmSS;
// import frc.robot.subsystems.ElevatorSS;
// import frc.robot.subsystems.WristSS;
// import frc.robot.subsystems.InfeedSS;

// public class L1ToggleCoCommand extends SequentialCommandGroup{



//     public L1ToggleCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

//         addCommands(
//                 new ConditionalCommand(
//                     null,
//                     null,
//                     () -> s_Arm.returnSetPoint() == ArmConstants.L1
//                 )
//         );
//         addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
//     }
    
   
// }