package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public PhotonCamera camera;

    public Vision(PhotonCamera cam) {
        camera = cam;
    }
    public Transform3d getFieldRelPos() {
        var result = camera.getLatestResult();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
            return fieldToCamera;
        }
        else {
            return new Transform3d(0,0,0,new Rotation3d(0,0,0
            ));
        }
        
    }
}
