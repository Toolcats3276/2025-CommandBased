package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.BaseCommands.Climber.ClimberIn;
import frc.robot.commands.BaseCommands.ElevatorCommands.ElevatorPIDCommand;
import frc.robot.commands.CompoundCommands.CancelCoCommand;
import frc.robot.commands.CompoundCommands.CompCoCommand;
import frc.robot.commands.CompoundCommands.ShootCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.AlgaeInfeedCycleCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.AlgaeInfeedSensorCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1CoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL1SensorCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.AlgaeInfeeds.ReefInfeedCommands.AlgaeInfeedL2SensorCoCommand;
import frc.robot.commands.CompoundCommands.AlgaeCommands.ScoringCommands.AlgaeToggleScoreCoCommand;
import frc.robot.commands.CompoundCommands.Climb.ClimbCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.CoralInfeedCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.CoralInfeedSensorCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.ToggleCoralInfeedCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.CoralInfeedCommands.ToggleCoralInfeedStateCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L1CoCommands.L1InverseCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L2CoCommands.L2ToggleCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L3CoCommands.L3ToggleCoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands.FrontL4CoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands.L4CoCommand;
import frc.robot.commands.CompoundCommands.CoralCommands.ScoreingCoCommands.L4CoCommands.L4ToggleCoCommand;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here. Carson was Here
 */
public class RobotContainer {

    private final SendableChooser<Command> AutoChooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController xboxController = new XboxController(1);
    // private final Joystick flightStick = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;
    private final int rotationAxis = Joystick.AxisType.kZ.value;

    private final int armAxis = XboxController.Axis.kLeftY.value;
    private final int elevatorAxisUp = XboxController.Axis.kLeftTrigger.value;
    private final int elevatorAxisDown = XboxController.Axis.kRightTrigger.value;
    private final int wristAxis = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 11);
    private final JoystickButton robotCentric = new JoystickButton(driver, 17);
    private final JoystickButton autoAlign = new JoystickButton(driver, 17);

    private final JoystickButton Comp = new JoystickButton(driver, 2);
    private final JoystickButton Cancel = new JoystickButton(driver, 10);
    //Reef
    private final JoystickButton L1 = new JoystickButton(driver, 9);
    private final JoystickButton L2 = new JoystickButton(driver, 8);
    private final JoystickButton L3 = new JoystickButton(driver, 6);
    private final JoystickButton L4 = new JoystickButton(driver, 7);
    //Barge
    private final JoystickButton Barge = new JoystickButton(driver, 5);
    //Feeds
    private final JoystickButton CoralInfeed = new JoystickButton(driver, 3);

    private final JoystickButton AlgaeInfeed = new JoystickButton(driver, 4);
    private final JoystickButton AlgaeInfeedL1 = new JoystickButton(driver, 14);
    private final JoystickButton AlgaeInfeedL2 = new JoystickButton(driver, 15);


    // private final JoystickButton TestPoint1 = new JoystickButton(driver, 14);
    // private final JoystickButton TestPoint2 = new JoystickButton(driver, 15);


    private final JoystickButton Shoot = new JoystickButton(driver, 1);



    private final JoystickButton Co_Cancel = new JoystickButton(xboxController, XboxController.Button.kB.value);
    private final JoystickButton Co_ResetElevatorCount = new JoystickButton(xboxController, XboxController.Button.kA.value);

    private final JoystickButton Co_X = new JoystickButton(xboxController, XboxController.Button.kX.value);
    private final JoystickButton Co_Y = new JoystickButton(xboxController, XboxController.Button.kY.value);
    private final JoystickButton Co_Start = new JoystickButton(xboxController, XboxController.Button.kStart.value);
    private final JoystickButton Co_LeftStick = new JoystickButton(xboxController, XboxController.Button.kLeftStick.value);

    private final POVButton top_Left = new POVButton(driver, 315);
    private final POVButton middle_Left = new POVButton(driver, 270);
    private final POVButton bottom_Left = new POVButton(driver, 225);
    private final Trigger Left_Trigger = new Trigger(top_Left.or(middle_Left).or(bottom_Left));

    private final POVButton top_Right = new POVButton(driver, 45);
    private final POVButton middle_Right = new POVButton(driver, 90);
    private final POVButton bottom_Right = new POVButton(driver, 135);
    private final Trigger Right_Trigger = new Trigger(top_Right.or(middle_Right).or(bottom_Right));

    private final JoystickButton POVAcitvate = new JoystickButton(driver, 0);
    // private final JoystickButton arm_Up = new JoystickButton(driver, 3);
    // private final JoystickButton arm_Down = new JoystickButton(driver, 4);
    // private final JoystickButton elevator_Up = new JoystickButton(driver, 5);
    // private final JoystickButton elevator_Down = new JoystickButton(driver, 6);


    /* Subsystems */

    // private final LimelightAssistant m_leftLimelight = new LimelightAssistant("limelight", LimelightConstants.MEGA_TAG_2_DISABLED_STD_DEV, false);

    public static final SwerveSS s_Swerve = new SwerveSS();
    private final SensorSS s_Sensor = new SensorSS();
    private final ElevatorSS s_Elevator = new ElevatorSS();
    private final ArmSS s_Arm = new ArmSS();
    private final InfeedSS s_Infeed = new InfeedSS();
    private final WristSS s_Wrist = new WristSS();
    private final ClimberSS s_Climber = new ClimberSS();
    // private final PoseEstimation s_PoseEstimation = new PoseEstimation(s_Swerve, m_leftLimelight);
    // private final ReefAlignment s_ReefAlignment = new ReefAlignment(m_leftLimelight, s_PoseEstimation);
    // private final PoseEstimation s_PoseEstimation = new PoseEstimation(s_Swerve, m_leftLimelight);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> autoAlign.getAsBoolean(),
                s_Elevator
                
            )
        );

        NamedCommands.registerCommand("FrontL4", new FrontL4CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        NamedCommands.registerCommand("Back L4", new L4CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        NamedCommands.registerCommand("Shoot", new ShootCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor));
        NamedCommands.registerCommand("Comp", new CompCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        NamedCommands.registerCommand("Infeed", new CoralInfeedSensorCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            .until(() -> CoralInfeedSensorCommand.endCommand));
        
        
        AutoChooser = new SendableChooser<Command>();
    
        AutoChooser.setDefaultOption("None", new PrintCommand("carson is a brick"));

        // AutoChooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
        // AutoChooser.addOption("S Test Auto", new PathPlannerAuto("S Test Auto"));
        // AutoChooser.addOption("Week Zero", new PathPlannerAuto("Week Zero"));
        AutoChooser.addOption("4G", new PathPlannerAuto("4G"));
        AutoChooser.addOption("1P", new PathPlannerAuto("1P"));

        SmartDashboard.putData(AutoChooser);        

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // infeed.onTrue(new InfeedCommand(s_Infeed, 1));
        Shoot.onTrue(new ShootCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor));


        Comp.onTrue(new CompCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        // infeed.onTrue(new InfeedCommand(s_Infeed, InfeedConstants.INFEED));
        // CoralInfeed.onTrue(new CoralInfeedSensorCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
        //     .until(() -> s_Sensor.endCoralInfeedCommand()));
        // CoralInfeed.onTrue(new CoralSourceInfeedSensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
        //     .until(() -> s_Sensor.endCoralInfeedCommand()));


        // good infeed commandsVVVVVVV
        CoralInfeed.onTrue(new ToggleCoralInfeedCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor)
            .until(() -> s_Sensor.endCoralInfeedCommand())
            .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setInfeedState(false))));

        // CoralInfeed.onTrue(new InstantCommand(() -> s_Sensor.toggleInfeedState())
        //     .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setInfeedState(false))));
        CoralInfeed.onTrue(new ToggleCoralInfeedStateCoCommand(s_Sensor)
            .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setInfeedState(false))));

        AlgaeInfeed.onTrue(new AlgaeInfeedSensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            .until(() -> s_Sensor.algaeInfeedDelay()));
        AlgaeInfeedL1.onTrue(new AlgaeInfeedL1SensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            .until(() -> s_Sensor.algaeInfeedDelay()));
        // AlgaeInfeedL1.onTrue(new AlgaeInfeedL1CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor));
        //^^^^^^^^


        // CoralInfeed.onTrue(new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ELEVATOR_TEST_POINT));
        // AlgaeInfeed.onTrue(new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ELEVATOR_TEST_POINT_2));

        AlgaeInfeedL2.onTrue(new AlgaeInfeedL2SensorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            .until(() -> s_Sensor.algaeInfeedDelay()));
        // AlgaeInfeed.onTrue(new AlgaeInfeedSensorL1CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor)
            // .until(() -> s_Sensor.algaeInfeedDebouncer()));
        // AlgaeInfeed.onTrue(new ToggleAlgaeInfeedCoCommand(s_Arm, s_Infeed, s_Wrist, s_Elevator, s_Sensor, Comp)
        //     .until(() -> s_Sensor.endAlgaeInfeedCommand())
        //     .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND))));

        // AlgaeInfeed.onTrue(new ToggleAlgaeInfeedStateCoCommand(s_Sensor)
        //     .handleInterrupt(() -> new InstantCommand(() -> s_Sensor.setAlgaeInfeedState(InfeedConstants.ALGAE_INFEED_GROUND))));

        //Reef
        L1.onTrue(new L1InverseCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        L2.onTrue(new L2ToggleCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        L3.onTrue(new L3ToggleCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        L4.onTrue(new L4ToggleCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));

        // L2.onTrue(new L2CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        // L3.onTrue(new L3CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        // L4.onTrue(new L4CoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));

        // L2.onTrue(new L2InverseCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        // L3.onTrue(new L3InverseCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        // L4.onTrue(new L4InverseCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        
        //Barge
        // Barge.onTrue(new BargeCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        // Barge.onTrue(new ProcessorCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed));
        Barge.onTrue(new AlgaeToggleScoreCoCommand(s_Wrist, s_Arm, s_Elevator, s_Infeed, s_Sensor, Barge));

        Cancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Climber, s_Elevator, s_Infeed, s_Sensor, 
            () -> xboxController.getRawAxis(wristAxis),
            () -> -xboxController.getRawAxis(armAxis),
            () -> xboxController.getRawAxis(elevatorAxisUp),
            () -> xboxController.getRawAxis(elevatorAxisDown)
            )
        );

        Co_Cancel.onTrue(new CancelCoCommand(s_Wrist, s_Arm, s_Climber, s_Elevator, s_Infeed, s_Sensor,
            () -> xboxController.getRawAxis(wristAxis),
            () -> -xboxController.getRawAxis(armAxis),
            () -> xboxController.getRawAxis(elevatorAxisUp),
            () -> xboxController.getRawAxis(elevatorAxisDown)
            )
        );

        Co_ResetElevatorCount.onTrue(new InstantCommand(() -> s_Elevator.resetEncoder()));

        // arm_Midway.onTrue(new ArmPID(0.2, .2, .2, 0, 0));

        // autoAlign.onTrue(new InstantCommand(() -> TeleopSwerve.LLRotationPIDController.reset())
        //     .alongWith(new InstantCommand(() -> TeleopSwerve.LLStrafePIDController.reset())));
            
        // cancel.onTrue(new manualArmCommand(s_Arm, -xboxController.getRawAxis(armAxis))
        //     .alongWith(new PrintCommand("button")));


        Co_Start.onTrue(new InstantCommand(() -> s_Climber.setSpeed(0)));
        // Co_X.onTrue(new InstantCommand(() -> s_Climber.setSpeed(1)));
        Co_X.onTrue(new ClimberIn(s_Climber));
        Co_Y.onTrue(new InstantCommand(() -> s_Climber.setSpeed(-1)));
        Co_LeftStick.onTrue(new ClimbCoCommand(s_Wrist, s_Arm, s_Elevator, s_Climber));


        // Left_Trigger.onTrue(AutoBuilder.pathfindThenFollowPath(s_Swerve.getPathfindingPath(() -> true), AlignmentConstants.PATHFINDING_CONSTRAINTS));
        Left_Trigger.onTrue(new AlgaeInfeedCycleCoCommand(s_Sensor, Left_Trigger));

        // TestPoint1.onTrue(new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ELEVATOR_TEST_POINT, ElevatorConstants.MAX_PID_OUTPUT));
        // TestPoint2.onTrue(new ElevatorPIDCommand(s_Elevator, ElevatorConstants.ELEVATOR_TEST_POINT_2, ElevatorConstants.MAX_PID_OUTPUT));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return AutoChooser.getSelected();
    }
}
