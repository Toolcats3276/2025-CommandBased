package frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L3CoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BaseCommands.ArmCommands.ArmPIDCommand;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.BaseCommands.InfeedCommands.InfeedCommand;
import frc.robot.commands.BaseCommands.WristCommands.WristPIDCommand;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.ElevatorSS;
import frc.robot.subsystems.WristSS;
import frc.robot.subsystems.InfeedSS;

public class L3CoCommand extends SequentialCommandGroup{



    public L3CoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed) {

        addCommands(
            new ParallelCommandGroup(
                new ArmPIDCommand(s_Arm, ArmConstants.L3, ArmConstants.MAX_PID_OUTPUT),
                new WristPIDCommand(s_Wrist, WristConstants.L3, WristConstants.MAX_PID_OUTPUT),
                new ElevatorPIDCommand(s_Elevator, ElevatorConstants.L3, ElevatorConstants.MAX_PID_OUTPUT),
                new SequentialCommandGroup(
                    new WaitCommand(0.375),
                    new InfeedCommand(s_Infeed, 0.25),
                    new WaitCommand(0.2),
                    new InfeedCommand(s_Infeed, 0)
                )
            )
        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}