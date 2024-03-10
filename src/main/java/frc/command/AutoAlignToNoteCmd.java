// package frc.command;

// import org.photonvision.PhotonCamera;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotMap.DriveConstants;
// import frc.subsystems.Drivetrain;
// import frc.subsystems.Vision;

// public class AutoAlignToDonut extends Command{
//     double yaw; 
//     Vision camera;
//     Drivetrain drivetrain;
//     double speed = 0.1;
//     public AutoAlignToDonut(Drivetrain drivetrain, Vision camera) {
//         this.camera = camera;
//         this.drivetrain = drivetrain;

//         addRequirements(drivetrain, camera);
//     }

//     @Override
//     public void execute(){

//         yaw = camera.getYaw();

//          if (Math.abs(yaw) < 0.5) {
//             drivetrain.stopModules();
//         } else {
//             ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,
//                     0, speed * yaw);
//             SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
//             drivetrain.setModuleStates(moduleStates);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.stopModules();
//     }

// }
