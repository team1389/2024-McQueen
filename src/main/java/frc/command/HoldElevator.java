package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.ElevatorSubsystem;

public class HoldElevator extends Command{
    private ElevatorSubsystem elevator;

    public HoldElevator(ElevatorSubsystem elevator){
        this.elevator = elevator  ;
    }

    @Override
    public void execute(){
        elevator.HoldElevator();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }
}
