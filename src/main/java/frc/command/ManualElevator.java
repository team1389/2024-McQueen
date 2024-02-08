package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Elevator;
import frc.subsystems.Shooter;

public class ManualElevator extends Command{
    Elevator elevator;
    private double power;

    public ManualElevator(Elevator elevator, double power) {
        this.elevator = elevator;
        this.power = power;
        
        addRequirements(elevator);
    }
    
    @Override
    public void execute() {
        // elevator.controllerInterrupt = true;
        elevator.moveElevator(power);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.moveElevator(0);
    }
}
