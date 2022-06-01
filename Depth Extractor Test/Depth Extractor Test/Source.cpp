#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>


using namespace std;
using namespace cv;


Mat image;
Mat imgHSV;
Mat OutputImage;


int iLowH = 0;
int iHighH = 12;
int iLowS = 60;
int iHighS = 255;
int iLowV = 0;
int iHighV = 245;


int acc = 1;
int rows = 10;
int para1 = 5;
int para2 = 10;
int minRad = 5;
int maxRad = 70;

int x;
int y;

int width = 640;
int height = 480;

int input;


int threshold_value = 127;
int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_binary_value = 255;


int xFocus = width / 2;
int yFocus = height / 2;


//static void HSVthreshold(int, int, int, int, int, int, void*)
//{
//    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), OutputImage);
//}

#define M_PI 3.14159  /* pi */
double deg2rad = M_PI / 180;
double rad2deg = 180 / M_PI;
int rsResX = 640;
int rsResY = 480;
int rsResolutionVert = 480, rsResolutionHor = 640, rsFOVvert = 42, rsFOVhor = 69;
double focalHor = (0.5 * rsResolutionHor) / tan(0.5 * rsFOVhor * deg2rad);
double focalVert = (0.5 * rsResolutionVert) / tan(0.5 * rsFOVvert * deg2rad);

vector<double> pixel2angles(double x, double y)
{
    double hor, vert;
    vector<double> outp(2);
    hor = -(atan2(focalHor, (x - rsResX / 2)) * rad2deg - 90);
    vert = atan2(focalVert , (y - rsResY / 2)) * rad2deg - 90;

    outp[0] = hor;
    outp[1] = vert;
    return outp;
}

int main()
{
    // Contructing piplines and other stuff to receive data from the realsense camera.

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;     //for color

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;



    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);    //for color

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    for (int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }
    while (true)
    {
        while (waitKey(1) < 0)
        {
            frames = pipe.wait_for_frames();
            auto depth_frame = frames.get_depth_frame();
            auto color_frame = frames.get_color_frame();
            // Make sure that both depth and  color are present for calculations
            if (!depth_frame || !color_frame)
                continue;


            //Get color each frame
            //rs2::frame color_frame = frames.get_color_frame();

            // Creating OpenCV Matrix from a color image
            Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP); ///////////////////////////////////////////////////

            // cheking if an image was read
            if (color.empty())
            {
                cerr << "image was not generated !" << endl;
                return 1;
            }

            namedWindow("Display Image", WINDOW_AUTOSIZE);
            imshow("Display Image", color);

            auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
            auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

            auto depth_intrin = depth_profile.get_intrinsics();
            auto color_intrin = color_profile.get_intrinsics();
            auto depth2color_extrin = depth_profile.get_extrinsics_to(color_profile);
            auto color2depth_extrin = color_profile.get_extrinsics_to(depth_profile);


            waitKey();

            //Inspireret af dne her kilde med koden: https://docs.opencv.org/4.x/da/d0c/tutorial_bounding_rects_circles.html

            Mat greyOut, threshOut, cannyImg;

            //Extract the contours so that
            vector<vector<Point> > contours; //Each contour point is saved in the vector
            vector<Vec4i> hierarchy;

            

            blur(color, color, Size(3, 3));
            cvtColor(color, greyOut, COLOR_BGR2GRAY);
            threshold(greyOut, threshOut, threshold_value, max_binary_value, threshold_type);
            //imshow("Greyscale window", greyOut);
            imshow("Threshold window", threshOut);

            cv::Canny(color, cannyImg, 127, 2 * 127);
            imshow("Canny edge detection window", cannyImg);
            findContours(cannyImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
            //Fungrere ikke fordi det omhandler billede typen og det kan ikke korerespondere med vores billede type//findContours(color, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); //retr_tree er en mode som modtager alle contour og rekonstruere fuld hierarki ag nested contours, chain er en approximation method der er simple fordi det her tagr horisontale, vertikale og diagonale linejr og efterlader kun deres end points
            
            vector<vector<Point> > contours_poly(contours.size());
            vector<Rect> boundRect(contours.size());
            //contours.resize(contours.size());
            //contours_poly.resize(contours_poly.size());
            //boundRect.resize(contours.size());
            vector<Point> centroidBox; //vector samlet med alle kooordinaterne for de her edgeboxes


            int closestDist = 800;
            int closestX = 0;
            int closestY = 0;
            int closestObj = 0, distCalc = 0;


            for (size_t i = 0; i < contours.size(); i++)
            {
                approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                boundRect[i] = boundingRect(Mat(contours_poly[i]));

                Scalar c = Scalar(255, 0, 0);
                //if (boundRect[i].width > 30 && boundRect[i].height > 30) //Kun boundary boxes med en speciifk størrlse kan bruegs
                //{
                    drawContours(color, contours, (int)i, c, 1, 8, vector<Vec4i>(), 0, Point());
                    rectangle(color, boundRect[i].tl(), boundRect[i].br(), c, 2, 8, 0);

                    //Dette tegner centerpunktet, der er ækvivalent til disse edgeboxes
                    centroidBox.push_back(Point(boundRect[i].x + (boundRect[i].width / 2), boundRect[i].y + (boundRect[i].height / 2)));

                    if (centroidBox[i].x != 0.0 && centroidBox[i].y != 0.0)
                    {
                        distCalc = sqrt(pow(abs(xFocus - centroidBox[i].x), 2) + pow(abs(yFocus - centroidBox[i].y), 2));

                        cout << "distCalc berore if: " << distCalc << endl;
                        cout << "i: " << centroidBox[i] << endl;
                        if (distCalc < closestDist)
                        {
                            cout << "closest before: " << closestDist << endl;
                            closestDist = distCalc;
                            cout << "distCalc: " << distCalc << endl <<
                                "Centerpoint: " << centroidBox[i] << endl;
                            closestObj = i;
                            closestX = centroidBox[i].x;
                            closestY = centroidBox[i].y;
                        }
                    }
                //}
                //tegner de her boundary boxes
                else
                {
                    centroidBox.push_back(Point(0.0, 0.0)); //hvis det ikke har en specifik acceptabel størrelse så sæt dem til (0.0)
                }
            }
            
             //Highlight Focus Object
            Scalar red = Scalar(0, 0, 255);
            drawContours(color, contours, (int)closestObj, red, 1, 8, vector<Vec4i>(), 0, Point());
            rectangle(color, boundRect[closestObj].tl(), boundRect[closestObj].br(), red, 2, 8, 0);

            //Det her er der intet galt med!
            //setting the estimated pixel (red)
            for (int i = closestX - 5; i < closestX + 5; i++)
            {
                for (int j = closestY - 5; j < closestY + 5; j++)
                {
                    Vec3b& target = color.at<Vec3b>(j, i);

                    target[0] = 0;
                    target[1] = 0;
                    target[2] = 255;

                    // set pixel
                    color.at<Vec3b>(Point(i, j)) = target;
                }
            }


            float rgb_src_pixel[2] = { closestX, closestY }; // The RGB coordinate for the center of the marble
            float dpt_tgt_pixel[2] = { 0 }; // The depth pixel that has the best match for that RGB coordinate

            auto sensor = selection.get_device().first<rs2::depth_sensor>();
            auto scale = sensor.get_depth_scale();

            // Search along a projected beam from 0.1m to 10 meter. This can be optimized to the concrete scenario, e.g. if you know that the data is bounded within [min..max] range
            rs2_project_color_pixel_to_depth_pixel(dpt_tgt_pixel, reinterpret_cast<const uint16_t*>(depth_frame.get_data()), scale, 0.1f, 5,
                &depth_intrin, &color_intrin,
                &color2depth_extrin, &depth2color_extrin, rgb_src_pixel);


            // Verify that the depth correspondence is valid, i.e within the frame boundaries


            auto distance = depth_frame.get_distance(dpt_tgt_pixel[0], dpt_tgt_pixel[1]);
            // Get the depth value for the pixel found

            //cout << "The distance to the object is: " << distance << endl;

            // Focus Pixel
            cout << "Measured Horizontal Focus Angle: " << 0 << endl <<
                "Measured Vertical Focus Angle: " << 0 << endl <<
                "x: " << xFocus << "   y: " << yFocus << endl;

            vector<double> objectAngles = pixel2angles(closestX, closestY);
            // Final Data For Detected Object
            cout << endl << endl << "The distance to the object is: " << distance << endl <<
                "Horizontal Angle to Object: " << objectAngles[0] << endl <<
                "Vertical Angle to Object: " << objectAngles[1] << endl;


            for (int i = xFocus - 5; i < xFocus + 5; i++)
            {
                for (int j = yFocus - 5; j < yFocus + 5; j++)
                {
                    Vec3b& target = color.at<Vec3b>(j, i);

                    target[0] = 102;
                    target[1] = 255;
                    target[2] = 0;

                    // set pixel
                    color.at<Vec3b>(Point(i, j)) = target;
                }
            }


            imshow("Bounding boxes:", color);
            //for (int i = 0; i < centroidBox.size(); i++)
            //{
            //    if (boundRect[i].width > 30 && boundRect[i].height > 30)
            //    {
            //        cout << "Point:     x: " << centroidBox[i].x << " y: " << centroidBox[i].y << endl;
            //    }
            //}
        }
}
    return 0;
}