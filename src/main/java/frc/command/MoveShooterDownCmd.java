package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;

public class MoveShooterDownCmd extends Command{
     private ShooterSubsystem shooter;

    public MoveShooterDownCmd(ShooterSubsystem shooter){
        this.shooter = shooter;
        // addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.runWristDown();
 //       addCommand(new WaitCommand(5));
        
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopWrist();
    }
}
