///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV. 					  	      **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/

// ZED includes
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

#include "createPathOrFile.h"

using namespace sl;

cv::Mat slMat2cvMat(Mat& input);

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD1080;
    init_params.depth_mode = DEPTH_MODE_ULTRA;
    init_params.coordinate_units = UNIT_MILLIMETER;
    if (argc > 1) init_params.svo_input_filename.set(argv[1]);

//     Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Display help in console
//    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_FILL;//SENSING_MODE_STANDARD SENSING_MODE_FILL

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width;
    int new_height = image_size.height;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat imageL(new_width, new_height, MAT_TYPE_8U_C4);
    Mat imageR(new_width, new_height, MAT_TYPE_8U_C4);
    Mat depthL(new_width, new_height, MAT_TYPE_32F_C1);

    cv::Mat ocvImageL = slMat2cvMat(imageL);
    cv::Mat ocvImageR = slMat2cvMat(imageR);
    cv::Mat ocvDepthL = slMat2cvMat(depthL);

    ZedIoUtils zedIoUtils;
    zed.enableTracking();
    cv::imshow("Image", ocvImageL);
    cv::imshow("Depth", ocvDepthL/20000.0f);
    // Loop until 'q' is pressed
    char key = cv::waitKey();
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(imageL, VIEW_LEFT, MEM_CPU, new_width, new_height);
            zed.retrieveImage(imageR, VIEW_RIGHT, MEM_CPU, new_width, new_height);
            zed.retrieveMeasure(depthL, MEASURE_DEPTH, MEM_CPU, new_width, new_height);

            Pose pose;
            TRACKING_STATE trackingState  = zed.getPosition(pose, REFERENCE_FRAME_WORLD);
            float6pose float6Pose;
            if(trackingState == SUCCESS)
            {
                sl::float3 rvec;
                sl::float3 tvec;
                rvec = pose.getRotationVector();//Rodrigues vector------Angle axis representation.
                tvec = pose.getTranslation();

                for(int i=0; i<3; ++i)
                {
                    float6Pose[i] = rvec[i];
                    float6Pose[i+3] = tvec[i];
                }

                double confi = pose.pose_confidence;
                std::cout<<"pose.pose_confidence:   "<<confi<<std::endl;
                if(confi > 0)
                    float6Pose.setState(1);
            }

            zedIoUtils.saveFrame(ocvImageL, ocvDepthL, ocvImageR, float6Pose);
            double maxVal = zed.getDepthMaxRangeValue();
            double minVal = zed.getDepthMinRangeValue();
        }
        cv::imshow("Image", ocvImageL);
        cv::imshow("Depth", ocvDepthL/20000.0f);
        // Handle key event
        key = cv::waitKey();
    }

    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
