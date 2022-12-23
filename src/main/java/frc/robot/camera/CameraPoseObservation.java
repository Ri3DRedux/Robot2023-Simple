package frc.robot.camera;

import edu.wpi.first.math.geometry.Pose2d;

public class CameraPoseObservation {
    public double time;//timer.getFPGATimestamp() referenced time when the observation was made
    public Pose2d estFieldPose; //Estimated robot pose on the field at the time of the observation
    public double trustworthiness; //Factor between 1.0 (fully trustworthy) and 0.0 (complete garbage) for this measurement. Abitrary units.

    CameraPoseObservation(double time, Pose2d estFieldPose, double trustworthiness){
        this.time = time;
        this.estFieldPose =estFieldPose;
        this.trustworthiness = trustworthiness;
    }
}
