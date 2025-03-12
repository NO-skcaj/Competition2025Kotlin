#pragma once

#include <cstdio>
#include <span>
#include <sstream>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cameraserver/CameraServer.h>

#include <frc/TimedRobot.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class AprilTags : public frc2::SubsystemBase
{
    public:

        AprilTags()
        {
            SetName("AprilTags");

            SetSubsystem("AprilTags");

            // We need to run our vision program in a separate thread. If not, our robot
            // program will not run.
            std::thread visionThread(VisionThread);
            visionThread.detach();
        };
        
        // This method will be called once per scheduler run
        void Periodic()
        {

        };
        
        /// @brief
        static void VisionThread()
        {
            // Declaring AprilTagDetector:
            frc::AprilTagDetector detector;

            // look for tag36h11
            detector.AddFamily("tag36h11", ApriltagConstants::NumberOfBitsCorrected);

            // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
            // Source of magic numbers: (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
            frc::AprilTagPoseEstimator::Config poseEstConfig =
            {
                .tagSize = units::length::inch_t(ApriltagConstants::LengthOfTagsInches),
                .fx = ApriltagConstants::CameraWidthInPixels,     // The width of the camera in pixels
                .fy = ApriltagConstants::CameraHeightInPixels,    // The Hight of the camera in pixels
                .cx = ApriltagConstants::CameraCenterXInPixels,   // The center focus point of the camera x pos
                .cy = ApriltagConstants::CameraCenterXInPixels    // The center focus point of the camera y pos
            };

            // Making estimator to estimate the tag's possition wich somehow makes it more accurate.
            frc::AprilTagPoseEstimator estimator = frc::AprilTagPoseEstimator(poseEstConfig);

            // Get the USB camera from CameraServer
            cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

            // Set the resolution
            camera.SetResolution(ApriltagConstants::CameraResolutionWidth, ApriltagConstants::CameraResolutionHeight);

            // Get a CvSink. This is what actualy gets the frame from the camera
            cs::CvSink cvSink = frc::CameraServer::GetVideo();

            // Setup a CvSource. This will send images back to the Dashboard
            cs::CvSource outputStream = frc::CameraServer::PutVideo("Detected", ApriltagConstants::CameraResolutionWidth, ApriltagConstants::CameraResolutionHeight);

            /// @brief Matrix representing the image
            cv::Mat mat;

            /// @brief Matrix representing the image... but in grayscale. Who needs
            cv::Mat grayMat;

            /// @brief vector containing all the tags currently detected.
            std::vector<int> detectedTags;

            /// @brief color of the lines outlineing the april tags
            cv::Scalar outlineColor = cv::Scalar(0, 255, 0);

            /// @brief color of the cross in the center of the april tags
            cv::Scalar crossColor = cv::Scalar(0, 0, 255);

            while (true)
            {
                // Tell the CvSink to grab a frame from the camera and put it in the source mat.
                // If there is an error notify the output.
                if (cvSink.GrabFrame(mat) == 0) // If GrabFrame returns 0 then it couldn't get a frame from the camera.
                {
                    // Send the output the error.
                    outputStream.NotifyError(cvSink.GetError());
                    //std::cout << "***** ERROR: Not able to send stream" << std::endl;

                    // If we don't have a frame from the camera, continueing the while loop would be pointless
                continue;
                }

                // If GrabFrame(mat) didn't return 0, then mat now stores the current frame.

                // Make grayMat the same as mat(The current Frame), but in grayscale.
                cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

                /// @brief Stores a bunch of information on the grayMat's size, but we just use it for width and height.
                cv::Size grayMatSize = grayMat.size();
                frc::AprilTagDetector::Results detections = detector.Detect(grayMatSize.width, grayMatSize.height, grayMat.data);

                // Have not seen any tags yet, so detectedTags should not have anything in it.
                detectedTags.clear();

                // Putting the number of April Tags detected on SmartDashboard:
                frc::SmartDashboard::PutNumber("Detections: ", detections.size());

                // If there are no detections then there is no point in looping through all the items in it.
                if (detections.size() > 0)
                {
                    // Looping through all the detections. detection is the current detecion that the loop is on.
                    for (const frc::AprilTagDetection *detection : detections)
                    {
                        // We see a tag, and we need to remember that we have seen this tag, so we put all the tags we detect in this nice vector.
                        detectedTags.push_back(detection->GetId());

                        // draw lines around the tag
                        for (auto cornerIndex = 0; cornerIndex <= (ApriltagConstants::NumberOfAprilTagCorners - 1); cornerIndex++)
                        {
                            int nextCornerIndex = (cornerIndex + 1) % ApriltagConstants::NumberOfAprilTagCorners;

                            // Getting the first point that we will draw a line from
                            const frc::AprilTagDetection::Point corner = detection->GetCorner(cornerIndex);

                            // Gettig the second line that we will draw a line to
                            const frc::AprilTagDetection::Point nextCorner = detection->GetCorner(nextCornerIndex);

                            // Drawing the line between the two points(corners) that we just got:
                            line(mat, cv::Point(corner.x, corner.y), cv::Point(nextCorner.x, nextCorner.y), outlineColor, ApriltagConstants::AprilTagLineWitdh);
                        }

                        // mark the center of the tag
                        const frc::AprilTagDetection::Point c = detection->GetCenter();
                        int crossSize = 10;

                        // Drawing the cross in the center of the tag
                        line(mat, cv::Point(c.x - crossSize, c.y), cv::Point(c.x + crossSize, c.y), crossColor, 2);
                        line(mat, cv::Point(c.x, c.y - crossSize), cv::Point(c.x, c.y + crossSize), crossColor, 2);

                        // identify the tag, and putting the tag name next to the cross in the center of the April tag
                        putText(mat, std::to_string(detection->GetId()), cv::Point(c.x + crossSize, c.y), cv::FONT_HERSHEY_SIMPLEX, 1, crossColor, 3);

                        // determine pose
                        frc::Transform3d pose = estimator.Estimate(*detection);

                        // Putting the april tag possition on SmartDashboard:
                        frc::SmartDashboard::PutNumber("April Tag X: ", (double) pose.X());
                        frc::SmartDashboard::PutNumber("April Tag Y: ", (double) pose.Y());
                        frc::SmartDashboard::PutNumber("April Tag Z: ", (double) pose.Z());

                        // putting the April Tag's position a string so that we can put the string onto SmartDashboard:
                        std::stringstream dashboardString;
                        dashboardString << "Translation: " << units::length::to_string(pose.X())
                                        << ", " << units::length::to_string(pose.Y()) << ", "
                                        << units::length::to_string(pose.Z());

                        // Putting the April Tag's rotation into the string so that we can put the string onto SmartDashboard
                        frc::Rotation3d rotation = pose.Rotation();
                        dashboardString << "; Rotation: "
                                        << units::angle::to_string(rotation.X()) << ", "
                                        << units::angle::to_string(rotation.Y()) << ", "
                                        << units::angle::to_string(rotation.Z());

                        // Putting the april tag's rotation and position onto SmartDashboard:
                        frc::SmartDashboard::PutString("pose_" + std::to_string(detection->GetId()), dashboardString.str());
                    }
                }

                // string we will put detectedTags into so that we can put it onto SmartDashboard
                std::stringstream detectedTagsString;
                if (detectedTags.size() > 0)
                {
                    if (detectedTags.size() > 1)
                    {
                        // Copying detectedTags into detectedTagsString. We will put detectedTagsString onto SmartDashboard later.
                        std::copy(detectedTags.begin(), detectedTags.end() - 1, std::ostream_iterator<int>(detectedTagsString, ","));
                    }

                    detectedTagsString << detectedTags.back();
                }

            // Putting all of the detected tags onto smartDashboard:
            frc::SmartDashboard::PutString("tags", detectedTagsString.str());

            // Give the output stream a new image to display
            outputStream.PutFrame(mat);
            }
        };
        
};
