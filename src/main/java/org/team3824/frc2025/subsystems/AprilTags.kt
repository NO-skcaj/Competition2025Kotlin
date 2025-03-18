package org.team3824.frc2025.subsystems

import edu.wpi.first.apriltag.AprilTagDetection
import edu.wpi.first.apriltag.AprilTagDetector
import edu.wpi.first.apriltag.AprilTagPoseEstimator
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.CvSink
import edu.wpi.first.cscore.CvSource
import edu.wpi.first.cscore.UsbCamera
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Rotation3d
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread

import org.team3824.frc2025.ApriltagConstants

object AprilTags : SubsystemBase() {
    
    init {
        name = "AprilTags"
        setSubsystem("AprilTags")
        
        // We need to run our vision program in a separate thread
        // If not, our robot program will not run
        val visionThread = thread(start = true) { 
            visionThread() 
        }
    }
    
    // This method will be called once per scheduler run
    override fun periodic() {
        // Add any periodic behavior here
    }

    fun visionThread() {
        // Declaring AprilTagDetector
        val detector = AprilTagDetector()

        // Look for tag36h11
        detector.addFamily("tag36h11", ApriltagConstants.NumberOfBitsCorrected)

        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // Source of magic numbers: (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
        val poseEstConfig = AprilTagPoseEstimator.Config(
            ApriltagConstants.LengthOfTagsInches,  // tagSize
            ApriltagConstants.CameraWidthInPixels,    // fx
            ApriltagConstants.CameraHeightInPixels,   // fy
            ApriltagConstants.CameraCenterXInPixels, // cx
            ApriltagConstants.CameraCenterYInPixels  // cy
        )

        // Making estimator to estimate the tag's position which somehow makes it more accurate
        val estimator = AprilTagPoseEstimator(poseEstConfig)

        // Get the USB camera from CameraServer
        val camera = CameraServer.startAutomaticCapture()

        // Set the resolution
        camera.setResolution(
            ApriltagConstants.CameraResolutionWidth,
            ApriltagConstants.CameraResolutionHeight
        )

        // Get a CvSink. This is what actually gets the frame from the camera
        val cvSink = CameraServer.getVideo()

        // Setup a CvSource. This will send images back to the Dashboard
        val outputStream = CameraServer.putVideo(
            "Detected",
            ApriltagConstants.CameraResolutionWidth,
            ApriltagConstants.CameraResolutionHeight
        )

        // Matrix representing the image
        val mat = Mat()

        // Matrix representing the image... but in grayscale
        val grayMat = Mat()

        // List containing all the tags currently detected
        val detectedTags = mutableListOf<Int>()

        // Color of the lines outlining the april tags
        val outlineColor = Scalar(0.0, 255.0, 0.0)

        // Color of the cross in the center of the april tags
        val crossColor = Scalar(0.0, 0.0, 255.0)

        while (true) {
            // Tell the CvSink to grab a frame from the camera and put it in the source mat
            // If there is an error notify the output
            if (cvSink.grabFrame(mat).toInt() == 0) { // If grabFrame returns 0 then it couldn't get a frame from the camera
                // Send the output the error
                outputStream.notifyError(cvSink.error)

                // If we don't have a frame from the camera, continuing the while loop would be pointless
                continue
            }

            // If grabFrame(mat) didn't return 0, then mat now stores the current frame

            // Make grayMat the same as mat (the current Frame), but in grayscale
            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY)

            // Stores information on the grayMat's size, but we just use it for width and height
            val grayMatSize = grayMat.size()
            val detections = detector.detect(Mat(grayMatSize, grayMat.type()))

            // Clear detected tags for new frame
            detectedTags.clear()

            // Putting the number of April Tags detected on SmartDashboard
            SmartDashboard.putNumber("Detections: ", detections.size.toDouble())

            // If there are no detections then there is no point in looping through all the items in it
            if (detections.isNotEmpty()) {
                // Looping through all the detections
                for (detection in detections) {
                    // We see a tag, and we need to remember that we have seen this tag,
                    // so we put all the tags we detect in this nice list
                    detectedTags.add(detection.id)

                    // Draw lines around the tag
                    for (cornerIndex in 0 until ApriltagConstants.NumberOfAprilTagCorners) {
                        val nextCornerIndex = (cornerIndex + 1) % ApriltagConstants.NumberOfAprilTagCorners

                        // Getting the first point that we will draw a line from
                        val corner: Pair<Double, Double> = Pair(detection.getCornerX(cornerIndex), detection.getCornerY(cornerIndex))

                        // Getting the second line that we will draw a line to
                        val nextCorner: Pair<Double, Double> = Pair(detection.getCornerX(nextCornerIndex), detection.getCornerY(nextCornerIndex))

                        // Drawing the line between the two points (corners) that we just got
                        Imgproc.line(
                            mat,
                            Point(corner.first, corner.second),
                            Point(nextCorner.first, nextCorner.second),
                            outlineColor,
                            ApriltagConstants.AprilTagLineWitdh
                        )
                    }

                    // Mark the center of the tag
                    val c: Pair<Double, Double> = Pair(detection.centerX, detection.centerY)
                    val crossSize = 10

                    // Drawing the cross in the center of the tag
                    Imgproc.line(
                        mat,
                        Point(c.first - crossSize, c.second),
                        Point(c.first + crossSize, c.second),
                        crossColor,
                        2
                    )
                    Imgproc.line(
                        mat,
                        Point(c.first, c.second - crossSize),
                        Point(c.first, c.second + crossSize),
                        crossColor,
                        2
                    )

                    // Identify the tag, and putting the tag name next to the cross in the center of the April tag
                    Imgproc.putText(
                        mat,
                        detection.id.toString(),
                        Point(c.first + crossSize, c.second),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        crossColor,
                        3
                    )

                    // Determine pose
                    val pose = estimator.estimate(detection)

                    // Putting the april tag position on SmartDashboard
                    SmartDashboard.putNumber("April Tag X: ", pose.x)
                    SmartDashboard.putNumber("April Tag Y: ", pose.y)
                    SmartDashboard.putNumber("April Tag Z: ", pose.z)

                    // Putting the April Tag's position into a string for SmartDashboard
                    val dashboardString = StringBuilder()
                    dashboardString.append("Translation: ${pose.x.toString()}, ")
                        .append("${pose.y}, ")
                        .append(pose.z.toString())

                    // Putting the April Tag's rotation into the string for SmartDashboard
                    val rotation = pose.rotation
                    dashboardString.append("; Rotation: ")
                        .append("${rotation.x}, ")
                        .append("${rotation.y}, ")
                        .append(rotation.z.toString())

                    // Putting the april tag's rotation and position onto SmartDashboard
                    SmartDashboard.putString("pose_${detection.id}", dashboardString.toString())
                }
            }

            // String we will put detectedTags into so that we can put it onto SmartDashboard
            val detectedTagsString = StringBuilder()
            if (detectedTags.isNotEmpty()) {
                if (detectedTags.size > 1) {
                    // Join all tags except the last one with commas
                    detectedTagsString.append(
                        detectedTags.subList(0, detectedTags.size - 1).joinToString(",")
                    )
                    detectedTagsString.append(",")
                }

                // Add the last tag
                detectedTagsString.append(detectedTags.last())
            }

            // Putting all of the detected tags onto SmartDashboard
            SmartDashboard.putString("tags", detectedTagsString.toString())

            // Give the output stream a new image to display
            outputStream.putFrame(mat)
        }
    }
}