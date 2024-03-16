// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Autos;

// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.RobotMap.*;
// import frc.robot.util.AutoGenerator;

// /** Add your docs here. */
// public class Auto_2P_Mid extends AutoPaths{

//     @Override
//     public Command load(frc.util.AutoGenerator autos) {
//         String p1Name = "2P_Middle";
//         PathPlannerPath p1 = PathPlannerPath.fromPathFile(p1Name);
//         var alliance = DriverStation.getAlliance();
    
//         Pose2d startingPose = null;
//         if (alliance.isPresent()) {
//             if (alliance.get() == DriverStation.Alliance.Red){
//                 startingPose = p1.flipPath().getPreviewStartingHolonomicPose();
//             } else {
//                 startingPose = p1.getPreviewStartingHolonomicPose();
//             }
//         } 
//         // TODO Auto-generated method stub
//         //shoot, follow path & intake, shoot
//         return Commands.sequence(
//             autos.scoringSequence(.95, 3000);
//             autos.resetOdometry(startingPose),
//             autos.pathIntake(p1Name).withTimeout(3),
//             autos.scoringSequence(limelight.calculateShooterAngle(), 4000)
//         );
//     }
    
// }