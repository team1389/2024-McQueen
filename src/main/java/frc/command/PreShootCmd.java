package frc.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.ShooterSubsystem;
import frc.subsystems.IndexerSubsystem;
import frc.subsystems.IntakeSubsystem;

public class PreShootCmd extends Command{
     private ShooterSubsystem shooterSubsystem;
     private IndexerSubsystem indexerSubsystem;
     private IntakeSubsystem intakeSubsystem;
     int timer = 0;
    public PreShootCmd(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem){
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        // addRequirements(intakeSubsystem, indexerSubsystem, shooterSubsystem);
    }

    @Override
    public void execute(){
        indexerSubsystem.moveToShoot();
        intakeSubsystem.runIntake();
        timer += 1;
        //addCommand(new WaitCommand(5));        
    }

    @Override
    public void end(boolean interrupted){
        indexerSubsystem.stop();
        intakeSubsystem.stop();
        // shooterSubsystem.stop();
        timer = 0;
    }

    @Override
    public boolean isFinished(){
        return (timer>25);
        // SmartDashboard.putBoolean("IsFinished PreeShoot", true);
        // return !(intakeSubsystem.hitSensor());
    }

}
