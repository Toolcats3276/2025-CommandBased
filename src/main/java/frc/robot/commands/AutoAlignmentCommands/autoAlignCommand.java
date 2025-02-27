// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.AutoAlignmentCommands;

// import java.io.IOException;
// import java.util.function.BooleanSupplier;

// import org.json.simple.parser.ParseException;

// import com.pathplanner.lib.util.FileVersionException;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSS;

// public class autoAlignCommand extends Command {

//   private final SwerveSS s_Swerve;
//   private final BooleanSupplier alignLeft;

//   public autoAlignCommand(SwerveSS s_Swerve, BooleanSupplier alignLeft) {
//     this.s_Swerve = s_Swerve;
//     this.alignLeft = alignLeft;
//   }

//   @Override
//   public void initialize() {

//     try {
//       s_Swerve.setPathfindingPath();
//     } catch (FileVersionException | IOException | ParseException e) {
//       // TODO Auto-generated catch block
//       e.printStackTrace();
//     }
    
//   }

//   @Override
//   public void execute() {}

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
