package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Intake;

public class RunIntake extends Command{
    private Intake intake;
    public RunIntake(Intake theintake){
        this.intake = theintake;
        addRequirements(intake);
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
