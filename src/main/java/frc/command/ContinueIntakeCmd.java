package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.IntakeSubsystem;

public class ContinueIntakeCmd extends Command{
    private IntakeSubsystem intake;
    public ContinueIntakeCmd(IntakeSubsystem intake){
        this.intake = intake;
        // addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runIntake();
        
    }

    @Override
    public void end(boolean interrupted){

        intake.stop();
    }

}
