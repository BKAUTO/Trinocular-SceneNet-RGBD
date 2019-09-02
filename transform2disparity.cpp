#include <iostream>
#include <opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Binocular camera internal reference, unit/meter
    float baseline = 0.2f;
    float focus = 0.2f;
    //Read the depth PNGs
    for(size_t j=0;j<5;j++)
    {
         for(size_t i=0;i<75;i++)
        {
            String foldpath = "/home/wbk/Desktop/render_out_335/"+to_string(i*100)+"_"+to_string(j)+"_depth.png";
            String writepath = "/home/wbk/Desktop/disparity_out/"+to_string(i*100)+"_"+to_string(j)+"_disp.png";
            String heatmapWritepath = "/home/wbk/Desktop/heatmap_out/"+to_string(i*100)+"_"+to_string(j)+"_heatmap.png";
            Mat img = imread(foldpath, IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
            //imshow(filenames[i],img);
            cout<<foldpath<<endl;
            int height = img.rows;//240
            int width = img.cols;//320
            Mat grayImg = imread(foldpath, IMREAD_GRAYSCALE);
            normalize(grayImg, grayImg, 0, 255, NORM_MINMAX);
            Mat img_heat;
            grayImg.convertTo(img_heat, CV_8UC3);

            applyColorMap(img_heat, img_heat, COLORMAP_RAINBOW);
            //imshow(foldpath,img_heat);
            //waitKey(1000);
            //Convert to float to compute
            Mat depthImg;
            img.convertTo(depthImg, CV_32F);
            depthImg = depthImg.mul(0.001);//transfer depth to meter

            //Generate the disparity map
            Mat disparityImg(1080,1280,CV_32F);

            for (int i=0;i<height;i++)
            {
                for(int j=0;j<width;j++){
                disparityImg.at<float>(i,j) = (baseline*focus)/depthImg.at<float>(i,j);
                if(disparityImg.at<float>(i,j)>(255/5542.566))
                    disparityImg.at<float>(i,j) = 0;
                }
            }

            disparityImg = disparityImg.mul(5542.566);//transfer disparity to pixel
            /*
            for (int i=0;i<height;i++)
            {
                for(int j=0;j<width;j++)
                cout<<disparityImg.at<float>(i,j)<<endl;
            }
            */

            //disparityImg.convertTo(disparityImg,CV_16U);
            imwrite(heatmapWritepath, img_heat);
            imwrite(writepath,disparityImg);
            //imshow(foldpath,disparityImg);
            //waitKey(1000);
        }
    }

/*
    Mat left = imread("/home/wbk/Desktop/render_out/4000_1_rgb.png", IMREAD_GRAYSCALE);
    Mat RGBleft = imread("/home/wbk/Desktop/render_out/4000_1_rgb.png", IMREAD_ANYCOLOR);
    circle(RGBleft, Point(800,600), 30, Scalar(0,20,25));
    imshow("4600_L_rgb.png", RGBleft);
    Mat right = imread("/home/wbk/Desktop/render_out/4000_0_rgb.png", IMREAD_GRAYSCALE);
    Mat RGBright = imread("/home/wbk/Desktop/render_out/4000_0_rgb.png", IMREAD_ANYCOLOR);
    imshow("4600_M_rgb.png", RGBright);
    Mat GroundTruth = imread("/home/wbk/Desktop/disparity_out/4000_1_disp.png", IMREAD_ANYDEPTH);
    Mat GroundTruth1;
    normalize(GroundTruth, GroundTruth1, 0, 255, NORM_MINMAX);
    imshow("GroudTruth.png", GroundTruth1);
    GroundTruth.convertTo(GroundTruth,CV_32F);

    Mat disp;
    int mindisparity = 0;
    int ndisparities = 160;
    int SADWindowSize = 15;
    //SGBM
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    int P1 = 8 * left.channels() * SADWindowSize* SADWindowSize;
    int P2 = 32 * left.channels() * SADWindowSize* SADWindowSize;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(11);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);
    //sgbm->setMode(cv::StereoSGBM::MODE_HH);
    sgbm->compute(left, right, disp);
    disp.convertTo(disp, CV_32F, 1.0 / 16);
    Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
    normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    cout<<"SGBM: "<<disp.at<float>(Point(800,600))<<endl;
    cout<<"GroundTruth: "<<GroundTruth.at<float>(Point(800,600))<<endl;
    imshow("SGBM.png", disp8U);
    imwrite("SGBM.png", disp);
    waitKey(0);
*/
    return 0;
}
