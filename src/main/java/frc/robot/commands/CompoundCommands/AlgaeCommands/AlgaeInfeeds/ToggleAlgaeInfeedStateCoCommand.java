package frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.InfeedConstants;
import frc.robot.subsystems.SensorSS;


public class ToggleAlgaeInfeedStateCoCommand extends SequentialCommandGroup{

    
    public ToggleAlgaeInfeedStateCoCommand(SensorSS s_Sensor) {

        addCommands(
                new ConditionalCommand(
                    //on true
                    new ConditionalCommand(
                        //on true
                        new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_L2)), 
                        //on false
                        new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_L1)), 
                        //condition
                            () -> s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L1
                    ), 

                    //on false
                    new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND)), 

                    //condition
                    () -> s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_GROUND ||
                    s_Sensor.getAlgaeInfeedState() == InfeedConstants.ALGAE_INFEED_L1
                    
                ).handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND)))
        );   
    }
    
    
} 