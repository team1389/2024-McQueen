package frc.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Shooter;

public class SetShoot extends Command{
    Shooter shooter;
    double rpm = 1000;
    public SetShoot(Shooter shooter){
        this.shooter = shooter;
        SmartDashboard.putNumber("Target RPM", rpm);
        //addRequirements(shooter);
    }

    @Override
    public void execute(){
        rpm = SmartDashboard.getNumber("Target RPM", rpm);
        // shooter.setShoot(rpm);
    }

    @Override
    public void end(boolean interrupted){
        shooter.setShoot(0);
    }
}