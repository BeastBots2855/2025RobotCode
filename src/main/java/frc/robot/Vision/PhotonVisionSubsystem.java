// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PhotonVisionSubsystem extends SubsystemBase {
  public final PhotonCamera frontLeft;
    public final PhotonCamera frontRight;
    public final PhotonCamera back;
    private PhotonPoseEstimator m_leftVisionPoseEstimator;
    private PhotonPoseEstimator m_rightVisionPoseEstimator;
    private PhotonPoseEstimator m_backVisionPoseEstimator;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    frontLeft = new PhotonCamera("frontLeft");
    frontRight = new PhotonCamera("frontRight");
    back = new PhotonCamera("back");

    

     try {
      m_leftVisionPoseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile), 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6), 
                -Units.inchesToMeters(14), 
                Units.inchesToMeters(19.5)), 
            new Rotation3d(0, 0,0)));

    } catch(IOException e){
      System.out.println(e.getMessage() + "\n april tags didnt load");
    }

    

    try {
      m_rightVisionPoseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile), 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6), 
                -Units.inchesToMeters(14), 
                Units.inchesToMeters(19.5)), 
            new Rotation3d(0, 0,0)));

    } catch(IOException e){
      System.out.println(e.getMessage() + "\n april tags didnt load");
    }

    try {
      m_backVisionPoseEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile), 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6), 
                -Units.inchesToMeters(14), 
                Units.inchesToMeters(19.5)), 
            new Rotation3d(0, 0,0)));

    } catch(IOException e){
      System.out.println(e.getMessage() + "\n april tags didnt load");
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
