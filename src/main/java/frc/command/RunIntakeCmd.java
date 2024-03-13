package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class RunIntakeCmd extends Command{
    private IntakeSubsystem intakeSubsystem;
    public RunIntakeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return intakeSubsystem.hitSensor();
    }
}
