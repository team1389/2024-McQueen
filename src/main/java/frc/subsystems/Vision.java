package frc.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    //find name later other wont work
    private final PhotonCamera cam;
    double yaw = 0;

    public Vision(){
        cam = new PhotonCamera("OV9281");
    }

    @Override
    public void periodic(){
        // var result = cam.getLatestResult();
        // PhotonTrackedTarget target = result.getBestTarget();
        // yaw = target.getYaw();
    }

    // public double getYaw() {
    //     return yaw;
    // }
    



}
