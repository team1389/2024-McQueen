// package frc.command;

// import java.util.function.Supplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotMap.DriveConstants;
// import frc.subsystems.Drivetrain;
// import frc.subsystems.LimelightVision;
// import frc.util.LimelightHelpers;

// public class TeleOpDrive extends Command {

//     private final Drivetrain drivetrain;
//     private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, flip;
//     private final Supplier<Boolean> fieldOrientedFunction, slowFunction, holdXFunction, BOOST, align;
//     private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
//     LimelightVision limeLightVision;
//     private double desiredAngle; // gyro value from getHeading() the robot wants to point at
//     private boolean holdingX = false;
//     private boolean lastHoldButton = false;
//     double tx;

//     public TeleOpDrive(Drivetrain drivetrain,
//             Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
//             Supplier<Double> rightY,
//             Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> slowFunction, Supplier<Boolean> holdXFunction, Supplier<Boolean> BOOST, Supplier<Boolean> align,  Supplier<Double> flip, LimelightVision limeLightVision)  {
//         this.drivetrain = drivetrain;
//         this.xSpdFunction = xSpdFunction;
//         this.ySpdFunction = ySpdFunction;
//         this.turningSpdFunction = turningSpdFunction;
//         this.fieldOrientedFunction = fieldOrientedFunction;
//         this.slowFunction = slowFunction;
//         this.holdXFunction = holdXFunction;
//         this.limeLightVision = limeLightVision;
//         this.BOOST = BOOST;
//         this.flip = flip;
//         this.align = align;

//         // A slew rate limiter caps the rate of change of the inputs, to make the robot
//         // drive much smoother
//         this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_LINEAR_ACCEL);
//         this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_LINEAR_ACCEL);
//         this.turningLimiter = new SlewRateLimiter(DriveConstants.MAX_ANGULAR_ACCEL);

//         this.desiredAngle = drivetrain.getHeading();

//         addRequirements(drivetrain,limeLightVision);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {

//         tx = LimelightHelpers.getTX("");
//         // 0. push desired angle to smart dashboard
//         SmartDashboard.putNumber("Desired Angle", desiredAngle);

//         // 1. Get real-time joystick inputs
//         double xSpeed = xSpdFunction.get();
//         double ySpeed = ySpdFunction.get();
//         double turningSpeed = turningSpdFunction.get();
//         boolean isAlign = align.get();
//         boolean holdButton = holdXFunction.get();

//         // 2. Apply deadband
//         xSpeed = Math.abs(xSpeed) > 0.07 ? xSpeed : 0.0;
//         ySpeed = Math.abs(ySpeed) > 0.07 ? ySpeed : 0.0;
//         turningSpeed = Math.abs(turningSpeed) > 0.07 ? turningSpeed : 0.0;

//         // 3. Make the driving smoother using the slew limiter
//         xSpeed = xLimiter.calculate(xSpeed * DriveConstants.MAX_METERS_PER_SEC);
//         ySpeed = yLimiter.calculate(ySpeed * DriveConstants.MAX_METERS_PER_SEC);
//         turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.MAX_RADIANS_PER_SEC);

//         // 4. Check right bumper for slow mode
//         if(flip.get() > 0.05) {
//             xSpeed *= 0.23;
//             ySpeed *= 0.23;
//             turningSpeed *= 0.325;
//         }
//         else if (slowFunction.get()) {
//             xSpeed *= 0.3;
//             ySpeed *= 0.3;
//             turningSpeed *= 0.325;
//         }
//         else if (!BOOST.get()) {
//             xSpeed *= 0.5;
//             ySpeed *= 0.5;
//             turningSpeed *= 0.5;
//         }

//         //4.5 auto aligning?
//         if(isAlign){
//             if (Math.abs(tx) < 0.5) {
//             turningSpeed = 0;
//             //rotAngle*speed
//         } else {
//             //You can tune 0.1
//             turningSpeed = tx * 0.1;
//         }
//         }

//         // 5. Construct desired chassis speeds
//         ChassisSpeeds chassisSpeeds;
//         if (fieldOrientedFunction.get()) {
//                 chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                     xSpeed, ySpeed, turningSpeed, new Rotation2d(drivetrain.getRotation2d().getRadians()));
//         } else {
//             // Relative to robot
//             chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
//         }

//         // 5.5 Figure out if we should hold X position
//         if (!lastHoldButton && holdButton) {
//             holdingX = !holdingX; // If the button was up and now it's down, toggle holding X
//         }
//         lastHoldButton = holdButton;

//         // 6. Convert chassis speeds to individual module states
//         SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);  
//         SmartDashboard.putNumber("FR target", (moduleStates[0].angle.getDegrees()));
//         SmartDashboard.putNumber("FL target", (moduleStates[1].angle.getDegrees()));
//         SmartDashboard.putNumber("BR target", (moduleStates[2].angle.getDegrees()));
//         SmartDashboard.putNumber("BL target", (moduleStates[3].angle.getDegrees()));
//         SmartDashboard.putBoolean("Field relative", fieldOrientedFunction.get());
//         SmartDashboard.putBoolean("Slow mode", slowFunction.get());
//         SmartDashboard.putBoolean("Holding X", holdingX);


//         // 7. Output all module states to wheels
//         if(!holdingX) { 
//             drivetrain.setModuleStates(moduleStates);
//         } else {
//             drivetrain.setX();
//         }
//     }   

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.stopModules();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }