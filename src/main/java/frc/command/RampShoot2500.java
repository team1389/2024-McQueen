package frc.command;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ShooterSubsystem;

public class RampShoot2500 extends Command{
    ShooterSubsystem shooter;
    Timer timer = new Timer();

    public RampShoot2500(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){

        shooter.runShoot(2500);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stop();
    }
}
