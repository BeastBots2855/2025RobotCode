/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.Vision;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class Vision {
    // private final static PhotonCamera plasticOrangePi = new PhotonCamera(VisionConstants.kPlasticOrangePi);
    private final static PhotonCamera metalOrangePiRED = new PhotonCamera(VisionConstants.kMetalOrangePiRED);
    private final static PhotonCamera metalOrangePiBLUE = new PhotonCamera(VisionConstants.kMetalOrangePiBLUE);
    
    // private final static PhotonPoseEstimator plasticOrangePiEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, VisionConstants.kRobotToPlasticTransform);
    private final static PhotonPoseEstimator metalOrangePiREDEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, VisionConstants.kRobotToMetalREDTransform);
   
    private final static PhotonPoseEstimator metalOrangePiBLUEEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, VisionConstants.kRobotToMetalBLUETransform);

    private static Matrix<N3, N1> curStdDevsPlastic = new Matrix<>(N3.instance, N1.instance);
    private static Matrix<N3, N1> curStdDevsMetalRED = new Matrix<>(N3.instance, N1.instance);
    private static Matrix<N3, N1> curStdDevsMetalBLUE = new Matrix<>(N3.instance, N1.instance);

 
     public Vision() {
            // plasticOrangePiEstimator.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
     }


    /**
     * Gets the estimated robot pose from the plastic orange Pi camera.
     * This may be empty if no targets are visible or the estimation fails.
     * 
     * @return An Optional containing the estimated robot pose, if available
     */
    // public static Optional<EstimatedRobotPose> getEstimatedPlasticPose(DriveSubsystem m_driveTrain) {
        
    //     Optional<EstimatedRobotPose> visionEst = Optional.empty();
    //     for (var result : plasticOrangePi.getAllUnreadResults()) {
    //         plasticOrangePiEstimator.addHeadingData(result.getTimestampSeconds(), new Rotation3d(m_driveTrain.getHeadingRotation2D()));
    //         visionEst = plasticOrangePiEstimator.update(result);
    //         if (visionEst.isPresent()) {
    //             updateEstimationStdDevs(plasticOrangePiEstimator, 
    //                 curStdDevsPlastic,
    //                 visionEst, 
    //                 result.getTargets());
    //         }
    //     }
    //     return visionEst;
    // }

    /**
     * Gets the estimated robot pose from the metal orange Pi RED camera.
     * This may be empty if no targets are visible or the estimation fails.
     * 
     * @param m_driveTrain The drive subsystem to get heading data from
     * @return An Optional containing the estimated robot pose, if available
     */
    public static Optional<EstimatedRobotPose> getEstimatedMetalREDPose(DriveSubsystem m_driveTrain) {
        
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : metalOrangePiRED.getAllUnreadResults()) {
            metalOrangePiREDEstimator.addHeadingData(result.getTimestampSeconds(), new Rotation3d(m_driveTrain.getHeadingRotation2D()));
            visionEst = metalOrangePiREDEstimator.update(result);
            if (visionEst.isPresent()) {
                updateEstimationStdDevsConstrained(metalOrangePiREDEstimator, 
                    curStdDevsMetalRED,
                    visionEst, 
                    result.getTargets());
            }
        }
        return visionEst;
    }

    /**
     * Gets the estimated robot pose from the metal orange Pi BLUE camera.
     * This may be empty if no targets are visible or the estimation fails.
     * 
     * @param m_driveTrain The drive subsystem to get heading data from
     * @return An Optional containing the estimated robot pose, if available
     */
    public static Optional<EstimatedRobotPose> getEstimatedMetalBLUEPose(DriveSubsystem m_driveTrain) {
        
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : metalOrangePiBLUE.getAllUnreadResults()) {
            metalOrangePiBLUEEstimator.addHeadingData(result.getTimestampSeconds(), new Rotation3d(m_driveTrain.getHeadingRotation2D()));
            visionEst = metalOrangePiBLUEEstimator.update(result);
            if (visionEst.isPresent()) {
                updateEstimationStdDevsConstrained(metalOrangePiBLUEEstimator, 
                    curStdDevsMetalBLUE,
                    visionEst, 
                    result.getTargets());
            }
        }
        return visionEst;
    }




    public static void addAllPoseEstimates(DriveSubsystem m_driveTrain,  SwerveDrivePoseEstimator m_drivePoseEstimator) {
        // var plasticPose = getEstimatedPlasticPose(m_driveTrain);
        var metalREDPose = getEstimatedMetalREDPose(m_driveTrain);
        var metalBLUEPose = getEstimatedMetalBLUEPose(m_driveTrain);

        // if (plasticPose.isPresent()) {
        //     m_drivePoseEstimator.addVisionMeasurement(
        //         plasticPose.get().estimatedPose.toPose2d(),
        //         plasticPose.get().timestampSeconds,
        //         curStdDevsPlastic);
        // }

        if (metalREDPose.isPresent()) {
            m_drivePoseEstimator.addVisionMeasurement(
                metalREDPose.get().estimatedPose.toPose2d(),
                metalREDPose.get().timestampSeconds,
                curStdDevsMetalRED);
        }

        if (metalBLUEPose.isPresent()) {
            m_drivePoseEstimator.addVisionMeasurement(
                metalBLUEPose.get().estimatedPose.toPose2d(),
                metalBLUEPose.get().timestampSeconds,
                curStdDevsMetalBLUE);
        }
    }



 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private static void updateEstimationStdDevs(
            PhotonPoseEstimator photonEstimator, 
            Matrix<N3, N1> curStdDevs, 
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = VisionConstants.kSingleTagStdDevs;
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = VisionConstants.kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = VisionConstants.kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;

                 if(avgDist < 2.5) {
                 curStdDevs = VecBuilder.fill(0.7, 0.7, 99999);
                 } else {
                    curStdDevs = VecBuilder.fill(10000, 100000, 99999);
                 }
             }
         }
     }












     private static void updateEstimationStdDevsConstrained(
        PhotonPoseEstimator photonEstimator, 
        Matrix<N3, N1> curStdDevs, 
        Optional<EstimatedRobotPose> estimatedPose, 
        List<PhotonTrackedTarget> targets) {
     if (estimatedPose.isEmpty()) {
         // No pose input. Default to single-tag std devs
         curStdDevs = VisionConstants.kSingleTagStdDevs;
     } else {
         // Pose present. Start running Heuristic
         var estStdDevs = VisionConstants.kSingleTagStdDevs;
         int numTags = 0;
         double avgDist = 0;

         // Precalculation - see how many tags we found, and calculate an average-distance metric
         for (var tgt : targets) {
             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
             if (tagPose.isEmpty()) continue;
             numTags++;
             avgDist +=
                     tagPose
                             .get()
                             .toPose2d()
                             .getTranslation()
                             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
         }

         if (numTags == 0) {
             // No tags visible. Default to single-tag std devs
             curStdDevs = VisionConstants.kSingleTagStdDevs;
         } else {
             // One or more tags visible, run the full heuristic.
             avgDist /= numTags;
             // Decrease std devs if multiple targets are visible
             if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
             // Increase std devs based on (average) distance
             if (numTags == 1 && avgDist > 4)
                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
             curStdDevs = estStdDevs;
         }
     }
 }







 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public static Matrix<N3, N1> getEstimatedPlasticSdtDevs() {
         return curStdDevsPlastic;
     }

    public static Matrix<N3, N1> getEstimatedMetalREDSdtDevs() {
        return curStdDevsMetalRED;
    }

    public static Matrix<N3, N1> getEstimatedMetalBLUESdtDevs() {
        return curStdDevsMetalBLUE;
    }

    
 
 

 }