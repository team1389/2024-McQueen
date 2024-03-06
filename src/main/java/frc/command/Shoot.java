package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Shooter;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;

public class Shoot extends Command{
     private Shooter shooter;
     private Intake intake;
     private double shootingPower = 0.5;

    public Shoot(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        SmartDashboard.putNumber("Shooting Power for Tuning", shootingPower);
        // addRequirements(shooter);
    }

//     @Override
//     public void execute(){
//         shooter.runShoot();
//  //       addCommand(new WaitCommand(5));
        
//     }

    @Override
    public void execute(){
        shootingPower = SmartDashboard.getNumber("Shooting Power for Tuning", shootingPower);
        shooter.runShoot(shootingPower);
 //       addCommand(new WaitCommand(5));
        
    }


    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }

    // @Override
    // public boolean isFinished(){
    //     return !intake.hitSensor();
    // }
}
