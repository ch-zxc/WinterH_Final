#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

Mat ROI_find(Rect RA, Rect R, Mat dilatemat, Rect ROI)
{
    Mat ROI_Mat;

    // printf("RA:%d %d %d %d\nR:%d %d %d %d", RA.x, RA.y, RA.width, RA.height, R.x, R.y, R.width, R.height);

    // Rect ROI(max(min(RA.x, R.x) - 50, 0), 
    //         max(min(RA.y, R.y) - 50, 0), 
    //         min(abs(RA.x - R.x) + max(RA.width, R.width) + 100, dilatemat.cols), 
    //         min(abs(RA.y - R.y) + max(RA.height, R.height) + 100, dilatemat.rows));

    // ROI_Mat = dilatemat(Range(ROI.x, ROI.y), Range(ROI.x + ROI.width, ROI.y + ROI.height));
    
    ROI_Mat = dilatemat(ROI);

    return ROI_Mat;
}

Point KFilter(Point RA, Point R ,Mat measurement, KalmanFilter KF)
{
    double center_x = (R.x + RA.x) / 2;
    double center_y = (R.y + RA.y) / 2;

    Mat prediction = KF.predict();
    Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

    measurement.at<float>(0) = (float)center_x;
    measurement.at<float>(1) = (float)center_y;
    KF.correct(measurement);

    //circle(ans, predict_pt, 3, Scalar(34, 255, 255), -1);

    center_x = (int)prediction.at<float>(0);
    center_y = (int)prediction.at<float>(1);

    return predict_pt;
}

Mat grphic_color_change(int mode, Mat imgOriginal)
{
    Mat imgHSV, imgBGR;
	Mat imgThresholded;

    if(0){
        vector<Mat> hsvSplit;                           //创建向量容器，存放HSV的三通道数据
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);   //Convert the captured frame from BGR to HSV
        split(imgHSV, hsvSplit);			            //分类原图像的HSV三通道
        equalizeHist(hsvSplit[2], hsvSplit[2]);         //对HSV的亮度通道进行直方图均衡
        merge(hsvSplit, imgHSV);				        //合并三种通道
        cvtColor(imgHSV, imgBGR, COLOR_HSV2BGR);        //将HSV空间转回至RGB空间，为接下来的颜色识别做准备
    }else{
        imgBGR = imgOriginal.clone();
        imgThresholded = imgOriginal.clone();
    }

    if(mode == 1){
        inRange(imgBGR, Scalar(78, 0, 0), Scalar(255, 255, 247), imgThresholded);
    }else{
        inRange(imgBGR, Scalar(0, 0, 78), Scalar(247, 255, 255), imgThresholded);
    }

    for (int i = 0; i < imgThresholded.rows; ++i){
        for (int j = 0; j < imgThresholded.cols; ++j){
            uchar pixel = imgThresholded.at<uchar>(i, j);
            
            // 判断当前像素是否为白色
            bool isWhitePixel = (pixel >= 250);
            
            // 反转效果 如果是白色像素则设置为黑色
            if(isWhitePixel){
                imgBGR.at<Vec3b>(i, j)[0] = 0;   // B通道
                imgBGR.at<Vec3b>(i, j)[1] = 0;   // G通道
                imgBGR.at<Vec3b>(i, j)[2] = 0;   // R通道
                //printf("yes %d %d\n",i ,j);
            }
        }
    }
																	
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
    imshow("imgBGR", imgBGR);

    return imgBGR;
}

void pnp(RotatedRect pnp_rect)
{
    Point2f pnp_vertices[4];      //定义矩形的4个顶点
	pnp_rect.points(pnp_vertices); 

    vector<Point3f> Points3D;

    Points3D.push_back(Point3f(-67, -27.5, 0));
    Points3D.push_back(Point3f(-67, 27.5, 0));
    Points3D.push_back(Point3f(67, -27.5, 0));
    Points3D.push_back(Point3f(67, 27.5, 0));

    //装甲板角点
    vector<Point2f> Points2D;
    Points2D.push_back(pnp_vertices[1]); //4
    Points2D.push_back(pnp_vertices[2]); //3
    Points2D.push_back(pnp_vertices[3]); //1
    Points2D.push_back(pnp_vertices[4]); //2

    //SolvePnP
    Mat rvec = Mat::zeros(3, 1, CV_64F);
    Mat tvec = Mat::zeros(3, 1, CV_64F);

    Mat rvec1 = Mat::zeros(3, 1, CV_64F);
    Mat tvec1 = Mat::zeros(3, 1, CV_64F);

    Mat_<double> intrinstic_matrix = (Mat_<double>(3, 3) << 1000, 0, 500, 0, 1000, 300, 0, 0, 1);
    Mat_<double> distortion_vec = (Mat_<double>(1, 5) << -0.1, 0.02, 0, 0, -0.001);


    //CV_EXPORTS_W bool solvePnP( InputArray objectPoints, InputArray imagePoints,
    //                       InputArray cameraMatrix, InputArray distCoeffs,
    //                       OutputArray rvec, OutputArray tvec,
    //                       bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE );

    solvePnP(Points3D, Points2D, intrinstic_matrix, distortion_vec, rvec1, tvec1, false);
    //solvePnPRansec(Points3D, Points2D, intrinsic_matrix, distortion_vec, rvec, tvec, false, SOLVEPNP_AP3P)
    cout << rvec1 << " \n" << tvec1 << endl; 

    return;
}

int main()
{
    int mode = 2;//mode:1=red 2=blue 
    VideoCapture video_source("../1.mp4"); 
    Mat ans;
    double heights[16];
    int t = 0;
    bool flag = false;
    RotatedRect RA[16], R[16];
    Rect ROI_final;
    RotatedRect pnp_rect;
    int stateNum = 4; //状态量
    int measureNum = 2; //测量量
    KalmanFilter KF(stateNum, measureNum, 0); 

    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    setIdentity(KF.measurementMatrix);                      //H=[1,0,0,0;0,1,0,0] 测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));     //Q高斯白噪声，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); //R高斯白噪声，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));           //P后验误差估计协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));  //初始化状态为随机值 其实就是x_0
    
    int ROI_time = 0, ROI_not_time = 0;
    bool ROI_mode = false;

    for (int all = 0; ; ++all){
        Mat sec, ROI_MAT, ROI_Back_MAT;
        int mark;
        video_source >> sec;    
        imshow("sec", sec);
        Mat source_Mat = sec;
        inRange(sec, Scalar(0, 0, 0), Scalar(0, 0, 0), source_Mat);
        inRange(source_Mat, Scalar(10, 10, 10), Scalar(10, 10, 10), source_Mat);
        imshow("source", source_Mat);
        Mat color_change = grphic_color_change(mode, sec);
        ans = sec;

        Mat grey;
        cvtColor(color_change, grey , COLOR_BGR2GRAY);
        imshow("grey", grey);
        
        Mat thresh_binary;
        threshold(grey, thresh_binary, 110, 255, THRESH_BINARY); //best light_borad
        //imshow("thresh_binary", thresh_binary);

        //腐蚀 & 膨胀
        Mat dilatemat, dilatema;
        dilate(thresh_binary, dilatema, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1), 3);
        erode(dilatema, dilatemat, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1));
        imshow("dilatemat", dilatemat);
        
        if(ROI_mode){
            Mat ROI_change = source_Mat.clone();
            
            for (int i = ROI_final.x; i < ROI_final.x + ROI_final.width; ++i){
                for (int j = ROI_final.y; j < ROI_final.y + ROI_final.height; ++j){
                    ROI_change.at<uchar>(j, i) = dilatemat.at<uchar>(j, i);
                    bool isWhitePixel = (dilatemat.at<uchar>(j, i) >= 250);
        
                    // if (isWhitePixel) {
                    //     printf("ROI yes %d %d\n",j ,i);
                    // }

                    
                }
            }
            // Mat ROI_1 = dilatemat(Range(max(min(RA[mark].boundingRect().x, R[mark].boundingRect().x) - 50, 0), min(abs(RA[mark].boundingRect().x - R[mark].boundingRect().x) + max(RA[mark].boundingRect().width, R[mark].boundingRect().width) + 100, dilatemat.cols)), 
            // Range(max(min(RA[mark].boundingRect().y, R[mark].boundingRect().y) - 50, 0), min(abs(RA[mark].boundingRect().x - R[mark].boundingRect().x) + max(RA[mark].boundingRect().width, R[mark].boundingRect().width) + 100, dilatemat.cols)));

            // ROI_1.copyTo(ROI_change);
            

            dilatemat = ROI_change.clone();
            
        }
        imshow("dilatemat after ROI", dilatemat);

        vector<vector<Point>> contours;
        findContours(dilatemat, contours, RETR_LIST, CHAIN_APPROX_NONE);

        for (size_t i = 0; i < contours.size(); ++i){
            vector<Point> points;
            double area = contourArea(contours[i]);
            if (area < 60 || 1e3 < area){
                continue;
            }
            drawContours(dilatemat, contours, static_cast<int>(i), Scalar(0), 2);
            

            double high;
            points = contours[i];

            RotatedRect rrect = fitEllipse(points);
            Point2f* vertices = new Point2f[4];
            rrect.points(vertices);

            // vector<vector<Point>> contours_ploy(contours.size());
            // vector<Rect> boundRect(contours.size());  
            
            // 画矩形
            // for(int k = 0; k < contours.size(); ++k){
            //     //printf("%d\n",i);
            //     auto peri = arcLength(contours[k], true);
            //     approxPolyDP(contours[k], contours_ploy[k], 0.02* peri, true);
            //     //polylines(ans, contours_ploy[k], true, (0, 0, 255), 2);
            //     boundRect[k] = boundingRect(contours[k]);
            //     //if(len(approx) != 4) continue;
            //     rectangle(ans, boundRect[k], Scalar(0, 0, 255), 2);
            // }    
            for (int j = 0; j < 4; ++j){
                line(ans, vertices[j], vertices[(j + 1) % 4], Scalar(0, 0, 255), 4);
            }
            high = rrect.size.height;
  
            for (size_t j = 1; j < contours.size(); ++j){
                vector<Point> pointsA;
                double area = contourArea(contours[j]);
                double area2 = contourArea(contours[i]);
                if(area / area2 > 2 || area / area2 < 0.5){
                    continue;
                }
                //if(area != area2) continue;
                if(area2 < 20 || 1e3 < area2){
                    continue;
                }
                if(area < 20 || 1e3 < area){
                    continue;
                }
                
                double highA, distance, slop;
                pointsA = contours[j];

                RotatedRect rrectA = fitEllipse(pointsA);

                slop = abs(rrect.angle - rrectA.angle);
                highA = rrectA.size.height;
                distance = sqrt((rrect.center.x - rrectA.center.x) * (rrect.center.x - rrectA.center.x) + (rrect.center.y - rrectA.center.y) * (rrect.center.y - rrectA.center.y));

                double max_height, min_height;
                if(rrect.size.height > rrectA.size.height){
                    max_height = rrect.size.height;
                    min_height = rrectA.size.height;
                }else{
                    max_height = rrectA.size.height;
                    min_height = rrect.size.height;
                }

                double line_x = abs(rrect.center.x - rrectA.center.x);//中心距离
                double difference = max_height - min_height;//长度差异
                double aim = distance / ((highA + high) / 2);//宽度差异
                double difference3 = abs(rrect.size.width - rrectA.size.width);//
                double height = (rrect.size.height + rrectA.size.height) / 200;//
                double slop_low = abs(rrect.angle + rrectA.angle) / 2;//
                
                if((aim < 3.0 - height && aim > 2.0 - height 
                    && slop <= 5 && difference <= 8 && difference3 <= 5 
                    &&(slop_low <= 30 || slop_low >=150) && line_x > 0.6 * distance)//小装甲板
                    || (aim < 5.0 - height && aim > 3.2 - height 
                    && slop <= 7 && difference <= 15 && difference3 <= 8 
                    && (slop_low <= 30 || slop_low >= 150) && line_x > 0.7 * distance)){//大装甲板

                    //if(difference > 6) continue;
                    heights[t] = (rrect.size.height + rrectA.size.height) / 2;
                    R[t] = rrect;
                    RA[t] = rrectA;
                    t++;
                    flag = true;
                }  
            }
        }

        double maxx = 0;
        
        for (int i = 0; i < t; ++i){     //多个目标存在，打更近装甲板，效果有待商榷
            if(heights[i] >= maxx){
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
            circle(ans, Point((R[mark].center.x + RA[mark].center.x) / 2, (R[mark].center.y + RA[mark].center.y) / 2), 40, Scalar(255, 255, 0), 4);

            circle(ans, KFilter(R[mark].center, RA[mark].center, measurement, KF), 3, Scalar(34, 255, 255), -1);
            //KFilter(R[mark].center, RA[mark].center, measurement, KF);
            
            Rect ROI_Rect(max(min(RA[mark].boundingRect().x, R[mark].boundingRect().x) - 25, 5), 
                        max(min(RA[mark].boundingRect().y, R[mark].boundingRect().y) - 25, 5), 
                        min(abs(RA[mark].boundingRect().x - R[mark].boundingRect().x) + max(RA[mark].boundingRect().width, R[mark].boundingRect().width) + 50, dilatemat.cols - max(min(RA[mark].boundingRect().x, R[mark].boundingRect().x) - 25, 5) - 5), 
                        min(abs(RA[mark].boundingRect().y - R[mark].boundingRect().y) + max(RA[mark].boundingRect().height, R[mark].boundingRect().height) + 50, dilatemat.rows - max(min(RA[mark].boundingRect().y, R[mark].boundingRect().y) - 25, 5) - 5));

            rectangle(ans, ROI_Rect, Scalar(0, 255, 0), 2);
            ROI_MAT = ROI_find(RA[mark].boundingRect(), R[mark].boundingRect(), dilatemat, ROI_Rect);
            ROI_final = ROI_Rect;

            Point2f pnpcenter((float)ROI_Rect.x + (float)(ROI_Rect.width / 2), (float)ROI_Rect.y + (float)(ROI_Rect.height / 2));
            Size2f pnpsize((float)ROI_Rect.width, (float)ROI_Rect.height);
            RotatedRect pnp_rect(pnpcenter, pnpsize, 0);
            //pnp_rect = boundingRect(ROI_Rect);
            pnp(pnp_rect);
            
            if(!ROI_MAT.empty()){
                ROI_time++;
                ROI_not_time = 0;
                ROI_mode = true;
                if(ROI_time > 15){
                    ROI_mode = false;
                    ROI_time = 0;
                }
            }else{
                ROI_not_time++;
                ROI_time = 0;
                if(ROI_not_time >= 2){
                    ROI_mode = false;
                    ROI_not_time = 0;
                    ROI_time = 0;
                }
            }
            printf("ROItime %d\n", ROI_time);
            imshow("ROI", ROI_MAT);
        }else{
            ROI_mode = 0;
        }
        imshow("final",ans);
    
        t = 0;
        flag = false;

        waitKey(-1);
    }
    
    return 0;
}