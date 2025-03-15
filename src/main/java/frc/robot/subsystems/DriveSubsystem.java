// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private final boolean isVisionEnabled = true;

  Field2d field = new Field2d();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getHeadingRotation2D(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }); 

  SwerveDrivePoseEstimator m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    getHeadingRotation2D(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(0.5)), // initiial was 0.05 for both on top and 0.5 for bottom, 0.05, 0.05, 0.65
    VecBuilder.fill(0.55, 0.55, Units.degreesToRadians(6)) // 0.5, 0.5, 5
                                                                              );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    
    System.out.println("starting auto configuration ::::::::::::::::");
       RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, feedForward)->driveRobotRelative(speeds),
        new PPHolonomicDriveController(
          AutoConstants.kPIDDrive,
          AutoConstants.kPIDRot
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this 
      );

      System.out.println("AutoBuilderConfigured");
      
    
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }


  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getHeadingRotation2D(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    if(isVisionEnabled){
      m_DrivePoseEstimator.update(
      getHeadingRotation2D(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

      Vision.addAllPoseEstimates(this, this.m_DrivePoseEstimator);
    }

    field.setRobotPose(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    if(isVisionEnabled) {
      return m_DrivePoseEstimator.getEstimatedPosition();
    } else {
    return m_odometry.getPoseMeters();
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getHeadingRotation2D(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    if(isVisionEnabled) {
      m_DrivePoseEstimator.resetPosition(
        getHeadingRotation2D(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    //xSpeed *= .25;
    //ySpeed *= .25;
    //rot *= .25;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeadingRotation2D())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeadingRotation2D() {
    return Rotation2d.fromDegrees(getHeading()
    );
  }
  

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState(),
      }
    );
  }


  public void driveRobotRelative(ChassisSpeeds speeds){
    double speedx = speeds.vxMetersPerSecond;
    double speedy = speeds.vyMetersPerSecond;
    double rot = speeds.omegaRadiansPerSecond;
    drive(speedx, speedy, rot, false);
    
  }











      /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance()
    {
      if (isRedAlliance())
      {
        zeroHeading();
        //Set the pose 180 degrees
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else
      {
        zeroHeading();
      }
    }



    private boolean isRedAlliance()
    {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }



        public Command driveToPose(Pose2d pose)
    {
      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
          3, 4.0,
          Math.PI * 2, Math.PI * 2);

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
          pose,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                      );
    }



    public Command driveToPoseSlowMode(Pose2d pose)
    {
      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
          3, 0.5,
          Math.PI * 2, Math.PI * 2 / 3);

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
          pose,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                      );
    }




    public Pose2d shiftPoseRobotRelative(Pose2d currentPose, Translation2d shift) {
      // Apply the translation in robot-relative terms, considering the current orientation
      Translation2d newTranslation = currentPose.getTranslation().plus(shift.rotateBy(currentPose.getRotation()));
      // Return the new pose with the updated translation but maintaining the same rotation
      return new Pose2d(newTranslation, currentPose.getRotation());
  }

    public Pose2d getNearestPole(AutoScoreConstants.Side side) {
        Pose2d currentPose = getPose();
        Pose2d closestPose = AutoScoreConstants.REEF_FACE_ARRAY[0];
        double closestDistance = 999999;
        for(Pose2d reefPose : AutoScoreConstants.REEF_FACE_ARRAY) {
          double delta_x = Math.abs(currentPose.getX() - reefPose.getX());
          double delta_y = Math.abs(currentPose.getY() - reefPose.getY());
          double distance = Math.hypot(delta_x, delta_y);
          if(distance < closestDistance) {
            closestDistance = distance;
            closestPose = reefPose;
          }
        }
        if(closestPose.equals(AutoScoreConstants.REEF_FACE_ONE)) {
          return side.equals(AutoScoreConstants.Side.LEFT) ?  AutoScoreConstants.PoleA : AutoScoreConstants.PoleB;
        } else if(closestPose.equals(AutoScoreConstants.REEF_FACE_TWO)) {
          return side.equals(AutoScoreConstants.Side.LEFT) ? AutoScoreConstants.PoleC : AutoScoreConstants.PoleD;
        } else if(closestPose.equals(AutoScoreConstants.REEF_FACE_THREE)) {
          return side.equals(AutoScoreConstants.Side.LEFT) ? AutoScoreConstants.PoleE : AutoScoreConstants.PoleF;
        } else if(closestPose.equals(AutoScoreConstants.REEF_FACE_FOUR)) {
          return side.equals(AutoScoreConstants.Side.LEFT) ? AutoScoreConstants.PoleG : AutoScoreConstants.PoleH;
        } else if(closestPose.equals(AutoScoreConstants.REEF_FACE_FIVE)) {
          return side.equals(AutoScoreConstants.Side.LEFT) ? AutoScoreConstants.PoleI : AutoScoreConstants.PoleJ;
        } else if(closestPose.equals(AutoScoreConstants.REEF_FACE_SIX)) {
          return side.equals(AutoScoreConstants.Side.LEFT) ? AutoScoreConstants.PoleK : AutoScoreConstants.PoleL;
        } else {
          return new Pose2d();
        }
    }

        //needed because otherwise command will precalcualte all the nearest pole values rather than on the fly, specifically the getNearestPole
    public Command driveToFirstAutoScorePose(AutoScoreConstants.Side side){
      Translation2d shiftBackward = new Translation2d(-0.5, 0);
      Pose2d nearestPole = getNearestPole(side);
      Pose2d initalPoseToPlanTo = shiftPoseRobotRelative(nearestPole, shiftBackward);
      System.out.println("Initial Target Pose: " + initalPoseToPlanTo.getX() + ", " + initalPoseToPlanTo.getY()+ ", " + nearestPole.getRotation().getRadians());
      return driveToPose(initalPoseToPlanTo);
    }

    public Command driveToSecondAutoScorePose(AutoScoreConstants.Side side, Translation2d coralOffset) {
      Pose2d nearestPole = getNearestPole(side);
      // offset depending on coral location in intake
      nearestPole = shiftPoseRobotRelative(nearestPole, coralOffset);
      return driveToPoseSlowMode(nearestPole);
    }

    




    

    
}
