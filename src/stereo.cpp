/*
   IRTES-SET laboratory
   Authors: You Li (liyou026@gmail.com)
   Descirption: This a sample code of my PhD works
*/

#include "stereo.hpp"
#include "opencv2/core/core.hpp"


using namespace cv;
using namespace std;


//Transform the disparity matrix to psedo-color display mat
void Psedo_Gray2Color(const cv::Mat & gray_mat, cv::Mat & color_mat)
{

        std::vector<cv::Mat> mat_vec;
            
        cv::Mat blue = cv::Mat(gray_mat.rows, gray_mat.cols, CV_8U);//blue
        cv::Mat green  = cv::Mat(gray_mat.rows, gray_mat.cols, CV_8U);//green
        cv::Mat red = cv::Mat(gray_mat.rows, gray_mat.cols, CV_8U);//red
        cv::Mat mask = cv::Mat(gray_mat.rows, gray_mat.cols, CV_8U);
        
        //calculate values for the three channels
        cv::subtract(cv::Scalar(255),gray_mat, blue);	// blue(I) = 255 - gray(I)
        gray_mat.copyTo(red);
        gray_mat.copyTo(green);
        cv::compare(green,128,mask,CV_CMP_GE);
        cv::subtract(cv::Scalar(255),green,green,mask);
        
        green.convertTo(green, -1, 2.0, 0.0);

        mat_vec.push_back(blue);
        mat_vec.push_back(green);
        mat_vec.push_back(red);
        //generate psedo-colored image by merging three channels

        cv::merge(mat_vec,color_mat);

}

/*---------  Calculate the disparity may by SGBM algorithm
* img_L, img_R ----------- Rectified left and right image 
* disp         ----------- The disparity result
* disp_show    ----------- The psedo-color disparity mat used for display
*/
void calDisparity_SGBM(const cv::Mat& img_L,const cv::Mat& img_R,cv::Mat& disp,cv::Mat& disp_show)
{
    cv::StereoSGBM sgbm;

    // set the parameters of sgbm
    int cn = 1;//number of channels
    int SADWindowSize = 0, numberOfDisparities = 0;
    
    numberOfDisparities = 80;
    sgbm.preFilterCap = 40;
    sgbm.SADWindowSize = 7;
    sgbm.P1 = 10*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 20*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = 80;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 48;
    sgbm.disp12MaxDiff = -1;
    sgbm.fullDP = true;

    //scaling the value into 0-255
    cv::Mat disp8;
    

    //copyMakeBorder(img_L, img_L1, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img_R, img_R1, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
    //Semi Global Block Matching, by OpenCV
    sgbm(img_L, img_R, disp);
    
    //displf = disp.colRange(80, img_L.cols);

    disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

    //sometimes, we need equalize the gray image, since the distribution is not even. This is for showing only.
    cv::Mat img_equalize,img_color;
    cv::equalizeHist(disp8,img_equalize);

    //false color image generating
    Psedo_Gray2Color(img_equalize, disp_show);
        
 }



void triangulate10D(const cv::Mat& img, const cv::Mat& disp, cv::Mat& xyz,
                   const double f, const double cx, const double cy, const double b,ROI3D roi)
{
    //test
    int stype = disp.type();
    int dtype = CV_32FC3;
    CV_Assert(stype == CV_16SC1);
    xyz.create(disp.size(),CV_MAKETYPE(dtype,10));

   //assign the effective elements of Q matrix

    int rows = disp.rows;
    int cols = disp.cols;

    double px,py,pz;

    //handling the missing values
    double minDisparity = FLT_MAX;
    cv::minMaxIdx(disp, &minDisparity, 0, 0, 0 );

    std::vector<float> _dbuf(cols*3+1);

    int x_max = roi.x_max;
    int y_max = roi.y_max;
    int z_max = roi.z_max;

    for(int i = 0;i<rows;i++)
    {
        const uchar* gray_ptr = img.ptr<uchar>(i);
        const short* disp_ptr = disp.ptr<short>(i);
        //float *dptr = dbuf;

        float *dptr = xyz.ptr<float>(i);

        for(int j = 0; j<cols; j++)
        {
            uchar intensity = gray_ptr[j];
            short d = disp_ptr[j];
            double pw = b/(1.0*static_cast<double>(d));
            px = ((static_cast<double>(j) -cx)*pw)*16.0f;
            py = ((static_cast<double>(i) -cy)*pw)*16.0f;
            pz = (f*pw)*16.0f;

            if(fabs(d-minDisparity) <= FLT_EPSILON )
            {
                px = 1.0/0.0;
                py = 1.0/0.0;
                pz = 1.0/0.0;
            }

            if(px>x_max || pz>z_max || py<y_max)//outside the ROI
            {
                dptr[j*10] = (float)px;   //X
                dptr[j*10 + 1] = (float)py; //Y
                dptr[j*10 + 2] = (float)pz; //Z
                dptr[j*10 + 3] = (float)j;  //u
                dptr[j*10 + 4] = (float)i;  //v
                dptr[j*10 + 5] = (float)d/16.0f; //disparity
                dptr[j*10 + 6] = (int)intensity;        //intensity
                dptr[j*10 + 7] = 0;        //I_u
                dptr[j*10 + 8] = 0;        //I_v
                dptr[j*10 + 9] = 0;        //motion mark
            }
            else                                //in the ROI
            {
                dptr[j*10] = (float)px;        //X
                dptr[j*10 + 1] = (float)py;      //Y
                dptr[j*10 + 2] = (float)pz;      //Z
                dptr[j*10 + 3] = (float)j;       //u
                dptr[j*10 + 4] = (float)i;       //v
                dptr[j*10 + 5] = (float)d/16.0f; //disparity
                dptr[j*10 + 6] = (int)intensity; //intensity
                dptr[j*10 + 7] = 0;           //I_u
                dptr[j*10 + 8] = 0;           //I_v
                dptr[j*10 + 9] = 0;           //motion mark

            }



        }
    }
}



/*
******this function rectify the coordinates of the 3D point cloud by the estimated pitch angle to the ground
******xyz --- the point cloud
******pitch1 --- the first estimated pitch value
 */
void correct3DPoints(cv::Mat& xyz, ROI3D& roi_, const double& pitch1, const double& pitch2)
{

  double cos_p1 = cos(pitch1);
  double sin_p1 = sin(pitch1);

  int cols = xyz.cols;
  int rows = xyz.rows;
    
  for(int j = 0; j < rows; j++)
  {

      float* xyz_ptr = xyz.ptr<float>(j);
      
      for(int i = 0;i < cols; i++)
      {
        float xp = xyz_ptr[10*i];
        float yp = xyz_ptr[10*i+1];
        float zp = xyz_ptr[10*i+2];

        int d = cvRound(xyz_ptr[10*i+5]);

        if(d < 25 && d>0)
        {
            xyz_ptr[10*i] = xp;
            xyz_ptr[10*i+1] = cos_p1 * yp + sin_p1 * zp;
            xyz_ptr[10*i+2] = cos_p1 * zp - sin_p1 * yp;

            if(xyz_ptr[10*i] > roi_.x_max || xyz_ptr[10*i+1] < roi_.y_max || xyz_ptr[10*i+2]>roi_.z_max) //outside the ROI
            {
                xyz_ptr[10*i+6] = 0;
            }


        }
        else if(d >= 25 && d<100)
        {
            xyz_ptr[10*i] = xp;
            xyz_ptr[10*i+1] = cos_p1 * yp + sin_p1 * zp;
            xyz_ptr[10*i+2] = cos_p1 * zp - sin_p1 * yp;

            if(xyz_ptr[10*i] > roi_.x_max || xyz_ptr[10*i+1] < roi_.y_max || xyz_ptr[10*i+2] > roi_.z_max) //outside the ROI
            {
                xyz_ptr[10*i+6] = 0;
            }
        }
        else
        {
             xyz_ptr[10*i+6] = 0;
        }

      }
    }

}

void setImageROI (cv::Mat& xyz, cv::Mat& roi_mask)
{
      vector<Mat> channels(8);
      split(xyz, channels);
      cv::Mat ch6;
      ch6 = channels[6];

      roi_mask.create(ch6.size(),CV_8UC1);
      cv::convertScaleAbs(ch6,roi_mask);
}

void calDisparity_BM(const cv::Mat& img_L,const cv::Mat& img_R,cv::Mat& disp,cv::Mat& disp_show)
{
    StereoBM bm;

    // set the parameters of sgbm
    int numberOfDisparities = 0;

    numberOfDisparities = 80;
    bm.state->preFilterCap = 60;
    bm.state->SADWindowSize = 11;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = 80;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;


    //scaling the value into 0-255
    cv::Mat disp8;

    //Semi Global Block Matching, by OpenCV
    bm(img_L, img_R, disp);

    disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

    //sometimes, we need equalize the gray image, since the distribution is not even. This is for showing only.
    cv::Mat img_equalize,img_color;
    cv::equalizeHist(disp8,img_equalize);

    //false color image generating
    Psedo_Gray2Color(img_equalize, disp_show);

 }
