package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.ShooterConstants;
import frc.subsystems.IntakeSubsystem;
import frc.subsystems.ShooterSubsystem;

public class ShootPIDCmd extends Command{
     private ShooterSubsystem shooterSubsytem;
    //  private Intake intakeSubsytem; TBD
     private double shootingRPM;

    public ShootPIDCmd(ShooterSubsystem shooterSubsytem, double setpoint){
        this.shooterSubsytem = shooterSubsytem;
        // this.intakeSubsytem = intakeSubsytem;
        shootingRPM = setpoint;
        SmartDashboard.putNumber("Shooting RPM for Tuning", shootingRPM);
        // addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ShooterPIDCmd started!");
    }


    @Override
    public void execute(){
        shootingRPM = SmartDashboard.getNumber("Shooting RPM for Tuning", shootingRPM);
        // shooterSubsytem.runShoot(shootingRPM);
        shooterSubsytem.runShoot(shootingRPM);        
    }


    @Override
    public void end(boolean interrupted){
        shooterSubsytem.stop();
        System.out.println("ShooterPIDCmd Ended!");
    }

    @Override
    public boolean isFinished(){
        // return !intake.hitSensor();
        return false;
    }
}
