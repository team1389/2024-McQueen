// package frc.subsystems;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Localization extends SubsystemBase{
//     private 
//     public Localization(){

//     }

//     public void updatePoseEstimatorWithVisionBotPose() {
//     PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
//     // invalid LL data
//     if (visionBotPose.pose2d.getX() == 0.0) {
//       return;
//     }

//     // distance from current pose to vision estimated pose
//     double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
//         .getDistance(visionBotPose.pose2d.getTranslation());

//     if (m_visionSystem.areAnyTargetsValid()) {
//       double xyStds;
//       double degStds;
//       // multiple targets detected
//       if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
//         xyStds = 0.5;
//         degStds = 6;
//       }
//       // 1 target with large area and close to estimated pose
//       else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
//         xyStds = 1.0;
//         degStds = 12;
//       }
//       // 1 target farther away and estimated pose is close
//       else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
//         xyStds = 2.0;
//         degStds = 30;
//       }
//       // conditions don't match to add a vision measurement
//       else {
//         return;
//       }

//       m_poseEstimator.setVisionMeasurementStdDevs(
//           VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
//       m_poseEstimator.addVisionMeasurement(visionBotPose.pose2d,
//           Timer.getFPGATimestamp() - visionBotPose.latencySeconds);
//     }
//   }
// }
