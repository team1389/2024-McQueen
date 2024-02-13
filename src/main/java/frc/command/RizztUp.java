package frc.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Indexer;
import frc.subsystems.Rizzt;

public class RizztUp extends Command{
    private Rizzt rizzt;
    boolean isShoot = true;


    public RizztUp(Rizzt rizt){
        this.rizzt = rizt;
        addRequirements(rizzt);
    }

    @Override
    public void execute(){
        rizzt.moveUp();
    }

    @Override
    public void end(boolean interrupted){
        rizzt.stop();
    }

}
