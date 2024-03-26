// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.FieldConstants;

public class NoteVision extends SubsystemBase {
    // Plot vision solutions
    public static final boolean PLOT_NOTES = true;
    private static final double ALLOWED_POSITION_ERROR = .2;

    // farthest away we can reliably see a NOTE
    public static final double MAX_VISIBLE_DISTANCE = 2.0; // meters

    // closest we can see a NOTE. Obscured by intake, or below field of view
    public static final double MIN_VISIBLE_DISTANCE = 0.2; // meters

    private static final String CAMERA_NAME = Constants.Vision.noteCameraName;
    private final PhotonCamera m_noteCamera = new PhotonCamera(CAMERA_NAME);

    // For Plotting
    private final Field2d m_notes = new Field2d();

    // Simulation support
    private VisionSystemSim m_visionSim;

    // Swerve for notes plotting
    private Swerve s_Swerve;

    public NoteVision(Swerve s_Swerve) {
        if (true /* Constants.USING_SIM */) {
            // initialize a simulated camera. Must be done after creating the tag layout
            initializeSimulation();
        }

        this.s_Swerve = s_Swerve;


        // set the driver mode to false
        m_noteCamera.setDriverMode(false);

        SmartDashboard.putData("Notes Poses", m_notes);
    }

    public void updateSimulation(Pose2d pose) {
        m_visionSim.update(pose);
    }

    // Get visible NOTEs in robot-centric coordinates
    public List<Translation2d> getNotes() {
        List<Translation2d> positions = new ArrayList<Translation2d>();

        if (!m_noteCamera.isConnected()) {
            SmartDashboard.putString("Note Camera: ", "Disconnected");
            return positions;
        } else {
            SmartDashboard.putString("Note Camera: ", "Connected");
        }

        var results = m_noteCamera.getLatestResult();
        List<PhotonTrackedTarget> targets = results.getTargets();

        for (PhotonTrackedTarget tgt : targets) {
            // this calc assumes pitch angle is positive UP, so flip the camera's pitch
            // note that PV target angles are in degrees
            double d = Math.abs(Constants.Vision.noteCamToRobot.getZ() /
                    Math.tan(-Constants.Vision.noteCamToRobot.getRotation().getY() + Math.toRadians(tgt.getPitch())));
            double yaw = Math.toRadians(tgt.getYaw());
            double x = d * Math.cos(yaw);
            double y = d * Math.sin(yaw);
            positions.add(new Translation2d(x, y));
        }
        return positions;
    }

    /*
     * Shouldn't need to use this. This is for debug. Use the get Notes field
     * relative function and pass in robot pose and use photonutils get rotation to
     * pose.
     */
    public List<Double> getNotesYaw() {
        List<Double> positions = new ArrayList<Double>();

        if (!m_noteCamera.isConnected()) {
            SmartDashboard.putString("Note Camera: ", "Disconnected");
            return positions;
        } else {
            SmartDashboard.putString("Note Camera: ", "Connected");
        }

        var results = m_noteCamera.getLatestResult();
        List<PhotonTrackedTarget> targets = results.getTargets();

        for (PhotonTrackedTarget tgt : targets) {
            double yaw = (tgt.getYaw());
            positions.add(yaw);
        }
        return positions;
    }

    // get visible NOTEs, in field-centric positions
    public List<Translation2d> getNotes(Pose2d robotPose) {
        List<Translation2d> positions = new ArrayList<Translation2d>();

        if (!m_noteCamera.isConnected()) {
            return positions;
        }

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotRotation = robotPose.getRotation().getRadians();

        for (PhotonTrackedTarget tgt : m_noteCamera.getLatestResult().getTargets()) {
            // this calc assumes pitch angle is positive UP, so flip the camera's pitch
            // note that PV target angles are in degrees
            double d = Math.abs(Constants.Vision.noteCamToRobot.getZ() /
                    Math.tan(-Constants.Vision.noteCamToRobot.getRotation().getY() + Math.toRadians(tgt.getPitch())));
            double yaw = Math.toRadians(tgt.getYaw());

            // the pi is because the camera is on the back
            double noteAngle = robotRotation - Math.PI + yaw;
            double fieldCentricNoteX = robotX + d * Math.cos(noteAngle);
            double fieldCentricNoteY = robotY + d * Math.sin(noteAngle);

            positions.add(new Translation2d(fieldCentricNoteX, fieldCentricNoteY));
        }
        return positions;
    }

    public boolean checkForNote(Pose2d robotPose, Translation2d wantedNote) {
        if (!m_noteCamera.isConnected()) {
            return false;
        }

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotRotation = robotPose.getRotation().getRadians();

        // goes through the found targets and checks if the wanted note pose is visible.
        for (PhotonTrackedTarget tgt : m_noteCamera.getLatestResult().getTargets()) {
            // this calc assumes pitch angle is positive UP, so flip the camera's pitch
            // note that PV target angles are in degrees
            double d = Math.abs(Constants.Vision.noteCamToRobot.getZ() /
                    Math.tan(-Constants.Vision.noteCamToRobot.getRotation().getY() + Math.toRadians(tgt.getPitch())));
            double yaw = Math.toRadians(tgt.getYaw());

            // the pi is because the camera is on the back
            double noteAngle = robotRotation - Math.PI + yaw;
            double fieldCentricNoteX = robotX + d * Math.cos(noteAngle);
            double fieldCentricNoteY = robotY + d * Math.sin(noteAngle);
            Translation2d notePosition = new Translation2d(fieldCentricNoteX, fieldCentricNoteY);

            if (notePosition.getDistance(wantedNote) <= ALLOWED_POSITION_ERROR) {
                return true;
            }
        }

        return false;
    }

    @Override
    public void periodic() {
        // DEBUG
        List<Translation2d> notes = getNotes();
        SmartDashboard.putNumber("noteVision nFound", notes.size());
        // if (notes.size() > 0) {
        //     m_notes.setRobotPose(FieldConstants.translationToPose2d(getNotes(s_Swerve.getEstimatedPose()).get(0)));
        //     SmartDashboard.putData("Notes Poses", m_notes);
        // }

    }

    private void initializeSimulation() {
        m_visionSim = new VisionSystemSim("NoteVision");

        // roughly our Logitech camera
        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(800, 600, Rotation2d.fromDegrees(90.0));
        prop.setFPS(30);
        prop.setAvgLatencyMs(10.0);
        prop.setLatencyStdDevMs(3.0);

        // Note: NetworkTables does not update the timestamp of an entry if the value
        // does not change.
        // The timestamp is used by PVLib to know if there is a new frame, so in a
        // simulation
        // with no uncertainty, it thinks that it is not detecting a tag if the robot is
        // static.
        // So, always have a little bit of uncertainty.
        prop.setCalibError(0.1, 0.03);

        PhotonCameraSim cam = new PhotonCameraSim(m_noteCamera, prop);
        cam.setMaxSightRange(Units.feetToMeters(15.0));
        m_visionSim.addCamera(cam, Constants.Vision.noteCamToRobot);

        // Add the Auto notes on the field
        TargetModel noteModel = new TargetModel(Units.inchesToMeters(14), Units.inchesToMeters(14),
                Units.inchesToMeters(2));
        for (Translation2d notePose : List.of(
                FieldConstants.NOTE_C_1,
                FieldConstants.NOTE_C_2,
                FieldConstants.NOTE_C_3,
                FieldConstants.NOTE_C_4,
                FieldConstants.NOTE_C_5,
                // Blue Stage Notes
                FieldConstants.BLUE_NOTE_S_1,
                FieldConstants.BLUE_NOTE_S_2,
                FieldConstants.BLUE_NOTE_S_3,
                // Red Stage Notes
                FieldConstants.RED_NOTE_S_1,
                FieldConstants.RED_NOTE_S_2,
                FieldConstants.RED_NOTE_S_3)) {
            m_visionSim.addVisionTargets("note",
                    new VisionTargetSim(new Pose3d(notePose.getX(), notePose.getY(), 0, new Rotation3d()), noteModel));
        }
    }

    // // --- Routines to plot the vision solutions on a Field2d ---------

    // private void clearTagSolutions(Field2d field) {
    // if (field == null)
    // return;
    // field.getObject("tagSolutions").setPoses();
    // field.getObject("visionPose").setPoses();
    // field.getObject("visionAltPose").setPoses();
    // field.getObject("visibleTagPoses").setPoses();
    // }

    // private void plotPose(Field2d field, String label, Pose2d pose) {
    // if (field == null)
    // return;
    // if (pose == null)
    // field.getObject(label).setPoses();
    // else
    // field.getObject(label).setPose(pose);
    // }

}