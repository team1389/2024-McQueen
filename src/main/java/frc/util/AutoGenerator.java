// package frc.util;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// import edu.wpi.first.wpilibj.DriverStation;
// import frc.subsystems.DriveSubsystem;
// import frc.subsystems.IndexerSubsystem;
// import frc.subsystems.IntakeSubsystem;
// import frc.subsystems.LimelightVisionSubsystem;
// import frc.subsystems.ShooterSubsystem;

// public class AutoGenerator {
//     private DriveSubsystem drivetrain;
//     IndexerSubsystem indexer;
//     IntakeSubsystem intake;
//     ShooterSubsystem shooter; 
//     LimelightVisionSubsystem limelight;

  
//   public AutoGenerator(DriveSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter) {
//     this.drivetrain = drivetrain;
//     this.indexer = indexer;
//     this.intake = intake;
//     this.shooter = shooter;
    
//     AutoBuilder.configureHolonomic(drivetrain::getPose, drivetrain::resetPose, drivetrain::getChassisSpeeds, drivetrain::driveRobotRelative,
//      new HolonomicPathFollowerConfig(new PIDConstants(2.55, 0.00008,0.0003), new PIDConstants(5.0, 0,0), Constants.Drivetrain.kMaxModuleSpeed, Constants.Drivetrain.kTrackRadius, new ReplanningConfig())
//     , ()->{
//       var alliance = DriverStation.getAlliance();
//                     if (alliance.isPresent()) {
//                         return alliance.get() == DriverStation.Alliance.Red;
//                     }
//                     return false;
//     }, drivetrain);


//     registerAllCommands();
   
    
//   }
// }
