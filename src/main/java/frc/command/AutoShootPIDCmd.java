package frc.command;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.LimelightVisionSubsystem;
import frc.subsystems.ShooterSubsystem;


public class AutoShootPIDCmd extends Command{
     private ShooterSubsystem shooterSubsytem;
     private LimelightVisionSubsystem limelightSubsystem;
     double rpm;
    Timer timer = new Timer();
    //double time;
    //  private Intake intakeSubsytem; TBD
     private double shootingRPM;

    public AutoShootPIDCmd(ShooterSubsystem shooterSubsytem, double rpm, LimelightVisionSubsystem limelightSubsystem){
        this.shooterSubsytem = shooterSubsytem;
        this.limelightSubsystem = limelightSubsystem;
        this.rpm = rpm;
        // this.intakeSubsytem = intakeSubsytem;
        SmartDashboard.putNumber("Shooting RPM for Tuning", shootingRPM);
        // addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        limelightSubsystem.blink();
    }


    @Override
    public void execute(){
        SmartDashboard.putNumber("Shooter time", timer.get());
        // shootingRPM = SmartDashboard.getNumber("Shooting RPM for Tuning", shootingRPM);
      //  shootingRPM = rpm;
        // shooterSubsytem.runShoot(shootingRPM);
        shooterSubsytem.runShoot(rpm);
    }

    @Override
    public void end(boolean interrupted){
        // shooterSubsytem.runShoot(shootingRPM);
        shooterSubsytem.stop();
    }

    @Override
    public boolean isFinished(){
        limelightSubsystem.off();
        return shooterSubsytem.isAtTargetRPM(rpm);
    }
}
