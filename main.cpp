#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

Mat ans;

void KFilter(Point RA, Point R , Mat measurement, KalmanFilter KF)
{
    double center_x = (R.x + RA.x) / 2;
    double center_y = (R.y + RA.y) / 2;

    Mat prediction = KF.predict();
    Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

    measurement.at<float>(0) = (float)center_x;
    measurement.at<float>(1) = (float)center_y;
    KF.correct(measurement);

    circle(ans, predict_pt, 3, Scalar(34, 255, 255), -1);

    center_x = (int)prediction.at<float>(0);
    center_y = (int)prediction.at<float>(1);

    return;
}

Mat grphic_color_change(int mode, Mat imgOriginal)
{
    Mat imgHSV, imgBGR;
	Mat imgThresholded;

    if(0)
    {
        vector<Mat> hsvSplit;   //创建向量容器，存放HSV的三通道数据
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        split(imgHSV, hsvSplit);			//分类原图像的HSV三通道
        equalizeHist(hsvSplit[2], hsvSplit[2]);    //对HSV的亮度通道进行直方图均衡
        merge(hsvSplit, imgHSV);				   //合并三种通道
        cvtColor(imgHSV, imgBGR, COLOR_HSV2BGR);    //将HSV空间转回至RGB空间，为接下来的颜色识别做准备
    }else{
        imgBGR = imgOriginal.clone();
        imgThresholded = imgOriginal.clone();
    }

    if(mode == 1){
        inRange(imgBGR, Scalar(78, 0, 0), Scalar(255, 255, 247), imgThresholded);
    }else{
        inRange(imgBGR, Scalar(0, 0, 78), Scalar(247, 255, 255), imgThresholded);
    }

    for (int i = 0; i < imgThresholded.rows; ++i) {
        for (int j = 0; j < imgThresholded.cols; ++j) {
            uchar pixel = imgThresholded.at<uchar>(i, j);
            
            // 判断当前像素是否为白色
            bool isWhitePixel = (pixel >= 250);
            
            // 如果是蓝色像素则设置为黑色
            if (isWhitePixel) {
                imgBGR.at<Vec3b>(i, j)[0] = 0;   // B通道
                imgBGR.at<Vec3b>(i, j)[1] = 0;   // G通道
                imgBGR.at<Vec3b>(i, j)[2] = 0;   // R通道
                printf("yes %d %d\n",i ,j);
            }
        }
    }
																	
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
    imshow("imgBGR", imgBGR);

    return imgBGR;
}

int main()
{
    int mode = 2;//mode:1=red 2=blue 
    VideoCapture video_source("../1.mp4"); 

    double heights[16];
    int t = 0;
    bool flag = false;
    RotatedRect RA[16], R[16];
    int stateNum = 4; //状态量
    int measureNum = 2; //测量量
    KalmanFilter KF(stateNum, measureNum, 0); 

    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值 其实就是x_0

    for(int all = 0; ; ++all){
        Mat sec;
        video_source >> sec;    
        imshow("sec",sec);
        Mat color_change = grphic_color_change(mode, sec);
        ans = sec;

        Mat grey;
        cvtColor(color_change, grey , COLOR_BGR2GRAY);
        imshow("grey", grey);
        
        Mat thresh_binary;
        threshold(grey, thresh_binary, 110, 255, THRESH_BINARY); //best light_borad
        //imshow("thresh_binary", thresh_binary);

        //腐蚀 & 膨胀
        Mat dilatemat,dilatema;
        dilate(thresh_binary, dilatema, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1), 3);
        erode(dilatema, dilatemat, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1));
        imshow("dilatemat",dilatemat);
        
        vector<vector<Point>> contours;
        findContours(dilatemat, contours, RETR_LIST, CHAIN_APPROX_NONE);

        for (size_t i = 0; i < contours.size(); ++i){
            vector<Point> points;
            double area = contourArea(contours[i]);
            if (area < 60 || 1e3 < area) continue;
            drawContours(dilatemat, contours, static_cast<int>(i), Scalar(0), 2);
            

            double high;
            points = contours[i];

            RotatedRect rrect = fitEllipse(points);
            Point2f* vertices = new Point2f[4];
            rrect.points(vertices);

            // vector<vector<Point>> contours_ploy(contours.size());
            // vector<Rect> boundRect(contours.size());  
            
            //画矩形
            // for(int k = 0; k < contours.size(); ++k){
            //     //printf("%d\n",i);
            //     auto peri = arcLength(contours[k], true);
            //     approxPolyDP(contours[k], contours_ploy[k], 0.02* peri, true);
            //     //polylines(ans, contours_ploy[k], true, (0, 0, 255), 2);
            //     boundRect[k] = boundingRect(contours[k]);
            //     //if(len(approx) != 4) continue;
            //     rectangle(ans, boundRect[k], Scalar(0, 0, 255), 2);
            // }    
            for (int j = 0; j < 4; j++)
            {
                line(ans, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255),4);
            }
            high = rrect.size.height;
  
            for(size_t j = 1; j < contours.size(); ++j){
                vector<Point> pointsA;
                double area = contourArea(contours[j]);
                double area2 = contourArea(contours[i]);
                if(area / area2 > 2 || area / area2 < 0.5) continue;
                //if(area != area2) continue;
                if(area2 < 20 || 1e3 < area2) continue;
                if(area < 20 || 1e3 < area) continue;
                
                double highA, distance, slop;
                pointsA = contours[j];

                RotatedRect rrectA = fitEllipse(pointsA);

                slop = abs(rrect.angle - rrectA.angle);
                highA = rrectA.size.height;
                distance = sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x) + (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));

                double max_height, min_height;
                if(rrect.size.height > rrectA.size.height){
                    max_height = rrect.size.height;
                    min_height = rrectA.size.height;
                }else{
                    max_height = rrectA.size.height;
                    min_height = rrect.size.height;
                }

                //从学长那里参考来的装甲板筛选（
                double line_x = abs(rrect.center.x-rrectA.center.x);//中心距离
                double difference = max_height - min_height;//长度差异
                double aim =   distance/((highA+high)/2);//宽度差异
                double difference3 = abs(rrect.size.width -rrectA.size.width);//
                double height = (rrect.size.height+rrectA.size.height)/200;//
                double slop_low = abs(rrect.angle + rrectA.angle)/2;//
                
                if((aim < 3.0 - height && aim > 2.0 - height 
                    && slop <= 5 && difference <= 8 && difference3 <= 5 
                    &&(slop_low <= 30 || slop_low >=150) && line_x >0.6*distance)//小装甲板
                    || (aim < 5.0 - height && aim > 3.2 - height 
                    && slop <= 7 && difference <=15 && difference3 <= 8 
                    && (slop_low <= 30 || slop_low >=150) && line_x >0.7*distance)){//大装甲板

                    //if(difference > 6) continue;
                    heights[t] = (rrect.size.height+rrectA.size.height)/2;
                    R[t] = rrect;
                    RA[t] = rrectA;
                    t++;
                    flag = true;
                }
                
                
            }
        }

        double maxx = 0;
        int mark;
        for(int i = 0;i < t;i++){     //多个目标存在，打更近装甲板
            if(heights[i]  >= maxx){
                maxx = heights[i];
                mark = i;
            }
        }

        // if(flag){
        //     for(int i = 0; i < t; ++i){

        //         circle(ans,Point((R[i].center.x + RA[i].center.x) / 2, (R[i].center.y + RA[i].center.y) / 2), 40, Scalar(255, 255, 0), 4);
        //         //string coordinatesText = to_string(static_cast<int>(i));
        //         //putText(ans, coordinatesText, Point((R[i].center.x + RA[i].center.x) / 2, (R[i].center.y + RA[i].center.y) / 2) , FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 0), 1, 4);
        //         //
        //         //putText();
        //     }
            
        // }
        if(flag){
            circle(ans,Point((R[mark].center.x + RA[mark].center.x) / 2, (R[mark].center.y + RA[mark].center.y) / 2), 40, Scalar(255, 255, 0), 4);
        
            KFilter(R[mark].center, RA[mark].center, measurement, KF);
        }
        imshow("final",ans);
    
        t = 0;
        flag = false;

        waitKey(-1);
    }
    
    return 0;
}