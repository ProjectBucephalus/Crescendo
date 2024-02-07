package frc.robot.commands.visioncommands;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class multiTagPoseEstimatior extends Command {
    
    private Vision s_Vision;
    public Transform3d pos;
    

    public multiTagPoseEstimatior (Vision s_Vision) {
        this.s_Vision = s_Vision;
        addRequirements(s_Vision);
    }

    @Override
    public void execute() {
        s_Vision.updateSwervePose();
    }
}
