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
     private double shootingRPM = 2000;

    public Shoot(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        SmartDashboard.putNumber("Left Shooter RPM Target:", shootingRPM);
        // addRequirements(shooter);
    }

//     @Override
//     public void execute(){
//         shooter.runShoot();
//  //       addCommand(new WaitCommand(5));
        
//     }

    @Override
    public void execute(){
        shootingRPM = SmartDashboard.getNumber("Left Shooter RPM Target:", shootingRPM);
        shooter.runShoot(shootingRPM);
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
