package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Shooter;
import frc.subsystems.Indexer;
import frc.subsystems.Intake;

public class Shoot extends Command{
     private Shooter shooter;
     private Intake intake;

    public Shoot(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        // addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.runShoot();
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
