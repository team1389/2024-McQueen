package frc.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    //find name later other wont work
    PhotonCamera camera;
    double yaw = 0;

    public Vision(PhotonCamera camera){
        this.camera = camera; 
    }

    @Override
    public void periodic(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        yaw = target.getYaw();
    }

    public double getYaw() {
        return yaw;
    }
    



}
