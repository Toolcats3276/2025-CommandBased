package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands;

import java.time.Instant;

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

public class AlgaeInfeedL2SensorInverseCoCommand extends SequentialCommandGroup{

    private boolean endCommand = false;

    public AlgaeInfeedL2SensorInverseCoCommand(WristSS s_Wrist, ArmSS s_Arm, ElevatorSS s_Elevator, InfeedSS s_Infeed, SensorSS s_Sensor) {

        addCommands(
            new RepeatCommand(
                new ConditionalCommand(
                    //on true
                        new ParallelCommandGroup(
                            // new SequentialCommandGroup(
                            //     new WaitCommand(0.375),
                            //     new WristPIDCommand(s_Wrist, WristConstants.COMP, WristConstants.ALGAE_INFEED_PID_OUTPUT),
                            //     new WaitCommand(0.5),
                            //     new ParallelCommandGroup(
                            //         new ArmPIDCommand(s_Arm, ArmConstants.ALGAE_INFEED_L2_COMP, ArmConstants.ALGAE_INFEED_PID_OUTPUT),
                            //         new ElevatorPIDCommand(s_Elevator, ElevatorConstants.COMP, ElevatorConstants.MAX_PID_OUTPUT)
                            //     ),
                            //     new InstantCommand(() -> endCommand = true)
                            //     ),
                            new InstantCommand(() -> s_Infeed.setVoltage(InfeedConstants.IDLE_ALGAE_VOLTAGE))
                        ),
                    //on false
                        new ParallelCommandGroup(
                            new InstantCommand(() -> endCommand = false),
                            new ArmPIDCommand(s_Arm, ArmConstants.ALGAE_INFEED_L2_Inverse, ArmConstants.MAX_PID_OUTPUT),
                            new WristPIDCommand(s_Wrist, WristConstants.ALGAE_INFEED_L2_Inverse, WristConstants.MAX_PID_OUTPUT),
                            new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ALGAE_INFEED_L2_Inverse, ElevatorConstants.MAX_PID_OUTPUT),
                            new InfeedCommand(s_Infeed, InfeedConstants.ALGAE_INFEED, InfeedConstants.ALGAE_INFEED)
                            // new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_L2))
                        ),
                    //condition 
                        () -> s_Sensor.algaeSensed()
                )
            ).until(() -> endCommand)

        );
        addRequirements(s_Wrist, s_Arm, s_Elevator, s_Infeed);
    }
    
   
}
