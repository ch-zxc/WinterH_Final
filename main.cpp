#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main()
{
    VideoCapture video_source("../1.mp4"); 

    double heights[16];
    int t = 0;
    bool flag = false;
    RotatedRect RA[16], R[16];

    for(int all = 0; ; ++all){
        Mat sec;
        video_source >> sec;      
        imshow("sec",sec);
        Mat ans = sec;

        Mat grey;
        cvtColor(sec, grey , COLOR_BGR2GRAY);
        imshow("grey", grey);
        
        Mat thresh_binary;
        threshold(grey, thresh_binary, 120, 255, THRESH_BINARY); //best light_borad
        //imshow("thresh_binary", thresh_binary);

        //腐蚀 & 膨胀
        Mat dilatemat,dilatema;
        dilate(thresh_binary, dilatema, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1), 3);
        erode(dilatema, dilatemat, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1));
        imshow("dilatemat",dilatemat);
        
        vector<vector<Point>> contours;
        findContours(dilatemat, contours, RETR_LIST, CHAIN_APPROX_NONE);

        for (int i = 0; i < contours.size(); ++i){
            vector<Point> points;
            double area = contourArea(contours[i]);
            drawContours(dilatemat, contours, static_cast<int>(i), Scalar(0), 2);

            double high;
            points = contours[i];

            RotatedRect rrect = fitEllipse(points);
            Point2f* vertices = new Point2f[4];
            rrect.points(vertices);

            vector<vector<Point>> contours_ploy(contours.size());
            vector<Rect> boundRect(contours.size());  
            
            //画矩形
            for(int k = 0; k < contours.size(); ++k){
                //printf("%d\n",i);
                auto peri = arcLength(contours[k], true);
                approxPolyDP(contours[k], contours_ploy[k], 0.02* peri, true);
                //polylines(ans, contours_ploy[k], true, (0, 0, 255), 2);
                boundRect[k] = boundingRect(contours[k]);
                //if(len(approx) != 4) continue;
                rectangle(ans, boundRect[k], Scalar(0, 0, 255), 2);
            }    
            high = rrect.size.height;
  
            for(int j = 1; j < contours.size(); ++j){
                vector<Point> pointsA;
                double area = contourArea(contours[j]);
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
                double line_x = abs(rrect.center.x-rrectA.center.x);
                double difference = max_height - min_height;
                double aim =   distance/((highA+high)/2);
                double difference3 = abs(rrect.size.width -rrectA.size.width);
                double height = (rrect.size.height+rrectA.size.height)/200;
                double slop_low = abs(rrect.angle + rrectA.angle)/2;
                
                if((aim < 3.0 - height && aim > 2.0 - height && slop <= 5 && difference <=8 && difference3 <= 5 &&(slop_low <= 30 || slop_low >=150) && line_x >0.6*distance)//小装甲板
                    || (aim < 5.0-height && aim > 3.2 - height && slop <= 7 && difference <=15 && difference3 <= 8 && (slop_low <= 30 || slop_low >=150) && line_x >0.7*distance)){//大装甲板

                    heights[t] = (rrect.size.height+rrectA.size.height)/2;
                    R[t] = rrect;
                    RA[t] = rrectA;
                    t++;
                    flag = true;
                }
            }
        }

        if(flag){
            for(int i = 0; i < t; ++i){
                circle(ans,Point((R[i].center.x + RA[i].center.x) / 2, (R[i].center.y + RA[i].center.y) / 2), 40, Scalar(255, 255, 0), 4);
                //putText(ans, "board", Point((R[i].center.x + RA[i].center.x) / 2, (R[i].center.y + RA[i].center.y) / 2) , FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 0), 1, 4);
            }
        }
        imshow("final",ans);
    
        t = 0;
        flag = false;

        waitKey(-1);
    }
    
    return 0;
}