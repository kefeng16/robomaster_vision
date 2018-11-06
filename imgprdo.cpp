#include "imgprdo.hpp"
#include "serial.h"
#include "angslu.hpp"
size_t _frame_count = 0;
double data_buff_X[10];
double data_buff_Y[10];

FileStorage fs("../config/camera640.xml", FileStorage::READ);
int deceted = 0;
int empty_fNUM = 0;
size_t frameCount = 0;
Rect rect;
myRect *ace = NULL;

double buff[3] = {0, 0, 0};

Mat produceImage(Mat *real, Rect *rect)
{
    Mat returnMat;
    returnMat = real->clone();
    int x, y, w, h;
    w = rect->width * 1.85;
    h = rect->height * 2.3;
    x = rect->x - w * 0.3 > 0 ? rect->x - w * 0.3 : 0;
    y = rect->y - h * 0.3 > 0 ? rect->y - h * 0.3 : 0;
    int rows = real->rows;
    int cols = real->cols;
    for (int i = 0; i < rows; i++)
    {
        Vec3b *pixelPtr1 = real->ptr<Vec3b>(i);
        Vec3b *pixelPtr2 = returnMat.ptr<Vec3b>(i);

        for (int j = 0; j < cols; j++)
        {
            if (!(i >= y && i <= y + h && j >= x && j <= x + w))
            {
                pixelPtr2[j][0] = 0;
                pixelPtr2[j][1] = 0;
                pixelPtr2[j][2] = 0;
            }
        }
    }
    return returnMat;
}

Mat findContourInEnemyColor(Mat *frame, color col, vector<RotatedRect> *rotateRects, vector<vector<Point>> *points)
{
    int filterRadius = 5;
    int filterSize = 2 * filterRadius + 1;
    double sigma = 10;
    vector<vector<Point>> smoothContours;
    Mat thre_gray_g, b_gry;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Mat> mats;
    split(*frame, mats);
    Mat mat_g = mats[1]; // - mats[1];
    Mat mat_b = mats[col == BLUE ? 0 : 2];
    threshold(mat_g, thre_gray_g, 170, 220, THRESH_BINARY);
    threshold(mat_b, b_gry, 170, 220, THRESH_BINARY);
    Mat fin = thre_gray_g & b_gry;

    /*潜龙勿用*/
    Mat structure = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    morphologyEx(fin, fin, MORPH_OPEN, structure);
    morphologyEx(fin, fin, MORPH_CLOSE, structure);
    GaussianBlur(fin, fin, Size(5, 5), 0, 0);
    findContours(fin, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    imshow("g", thre_gray_g);
    imshow("b", b_gry);
    imshow("&", fin);
    for (size_t i = 0; i < contours.size(); ++i)
    {
        RotatedRect r = minAreaRect(contours[i]);
        Point2f leftPoint[4];
        r.points(leftPoint);
        //   int ang = roatedRect_ang(&r);
        double val = getOrientation(contours[i]);
#ifdef DEBUG
        putText(*frame, to_string(val), r.center, 1, 0.7, Scalar(0, 255, 0));
        imshow("dddddddd", *frame);
        waitKey(1);

#endif

        // #define PRINT
        if (contours[i].size() < 15)
        {
#ifdef PRINT
            cout << "size"
                 << "\n";
#endif
            continue;
        }
        if (val > 120 || val < 70)
        {
#ifdef PRINT
            cout << "deg"
                 << "\n";
#endif
            continue;
        }
        if (checkOuter(&r, frame) != col)
            continue;
        rotateRects->push_back(r);
        points->push_back(contours[i]);
    }
    return thre_gray_g;
}

void ArmorDetection(Mat frame, int fd, color col)
{
    Mat cam_matrix_480, distortion_coeff_480;
    fs["Camera_Matrix"] >> cam_matrix_480;
    fs["Distortion_Coefficients"] >> distortion_coeff_480;
    AngleSolver solver_480(cam_matrix_480, distortion_coeff_480, 21.6, 4.4, 2.0, 0, 1000);
    AngleSolverFactory angle_slover;
    angle_slover.setTargetSize(21.6, 5.4, AngleSolverFactory::TARGET_ARMOR);
    angle_slover.setTargetSize(12.4, 5.4, AngleSolverFactory::TARGET_SAMLL_ATMOR);
    angle_slover.setSolver(&solver_480);
    Mat binaryFrame;
    vector<RotatedRect> all;
    vector<myRect> FinalArmas;
    vector<vector<Point>> points;
    Mat ROI; //= frame;
    int maxNum = 5;
    if (!deceted && (empty_fNUM > maxNum || frameCount == 0)) //上一次没有检测到并且差距较大
    {

        empty_fNUM++;
        binaryFrame = findContourInEnemyColor(&frame, col, &all, &points);
    }
    else if (!deceted && empty_fNUM <= maxNum)
    {
        empty_fNUM++;
        ROI = produceImage(&frame, &rect);
        binaryFrame = findContourInEnemyColor(&ROI, col, &all, &points);
        imshow("ROI", ROI);
    }
    else if (deceted && frameCount && ace != NULL) //上一次检测到了并且不是第一帧
    {

        ROI = produceImage(&frame, &rect);
        binaryFrame = findContourInEnemyColor(&ROI, col, &all, &points);
        empty_fNUM = 0;
        imshow("ROI", ROI);
    }

    vector<myRect> arams = make_allArma(&all, binaryFrame, &points);
    for (size_t i = 0; i < arams.size(); i += 2)
    {
        FinalArmas.push_back(arams[i]);
        // rectangle(frame, arams[i].right.boundingRect(), Scalar(0, 255, 0), 3);
        // rectangle(frame, arams[i].left.boundingRect(), Scalar(0, 255, 0), 3);
    }
    for (auto &FinalArma : FinalArmas)
    {
        computeArmaScore(&FinalArma);
    }

    ace = NULL;
    ace = chooseTargetInMyRectVec(&FinalArmas);

    if (ace != NULL)
    {
        _frame_count++;
#define LOG
#ifdef LOG
         //cout << " score :" << ace->score << "\n";
#endif
        deceted = 1;
        int rectx, recty, rectw, recth;
        RotatedRect leftr = ace->left.center.x < ace->right.center.x ? ace->left : ace->right;
        RotatedRect rigtr = ace->left.center.x > ace->right.center.x ? ace->left : ace->right;
        rectx = leftr.center.x < rigtr.center.x ? leftr.center.x : rigtr.center.x;
        recth = leftr.size.height > leftr.size.width ? leftr.size.height : leftr.size.width;
        recty = leftr.center.y - recth / 2;
        rectw = rigtr.center.x - leftr.center.x;
        rect.x = rectx;
        rect.y = recty;
        rect.width = rectw;
        rect.height = recth;
        Rect target(rectx, recty, rectw, recth);
        auto _x = rectx + (rectw / 2);
        auto _y = recty + (recth / 2);
        line(frame, Point(0, _y), Point(640, _y), Scalar(0, 255, 255));
        line(frame, Point(_x, 0), Point(_x, 480), Scalar(0, 255, 255));
        circle(frame, Point(_x, _y), 50, Scalar(0, 255, 0), 2);
        circle(frame, Point(_x, _y), 100, Scalar(255, 0, 255), 2);
        circle(frame, Point(_x, _y), 150, Scalar(255, 255, 0), 2);
        circle(frame, Point(_x, _y), 2, Scalar(0, 0, 255), 2);
        string text = "[" + to_string(_x) + "," + to_string(_y) + "]";
        putText(frame, text, Point(rectx, recty), 1, 0.8, Scalar(0, 255, 0));
        RotatedRect TARGET(Point2f(target.x, target.y), Point2f(target.x + target.width, target.y), Point2f(target.x + target.width, target.y + target.height));
        double __X = 0.0, __Y = 0.0;
        angle_slover.getAngle(TARGET, AngleSolverFactory::TARGET_SAMLL_ATMOR, __X, __Y, 22, 0);
        Data_filter(_frame_count, data_buff_X, &__X, &__X);
        Data_filter(_frame_count, data_buff_Y, &__Y, &__Y);
        buff[0] = __X * 0.8; //abs(__X) > 5 ? __X : 0;
        buff[1] = __Y ;
#ifdef LOG
        cout << " YAW:" << buff[0] << " PITCH:" << buff[1] << " Z:" << buff[2] << endl;
#endif
        if (fd != -1)
        {
            sendXYZ(fd, buff);
            // double buff[3] = {0, 0, 0};
            //sendXYZ(fd, buff);
        }
        rectangle(frame, target, Scalar(0, 255, 255), 2, 5);
    }

    else
    {
        _frame_count = 0;
        double buff[3] = {0, 0, 0};
        if (fd != -1)
        {
            sendXYZ(fd, buff);
        }
#ifdef LOG

        cout << " X:" << buff[0] << " Y:" << buff[1] << " Z:" << buff[2] << endl;
#endif
        deceted = 0;
    }
    frameCount++;
}
/*0 for blue, 1 for red*/
color checkOuter(RotatedRect *r, Mat *img)
{
    if (r != NULL && img != NULL)
    {
        try
        {
            //imshow("img", *img);
            float width = r->size.width < r->size.height ? r->size.width : r->size.height;
            float height = r->size.width > r->size.height ? r->size.width : r->size.height;
            size_t red_pix_c = 1, blue_pix_c = 1, pixNUM = 1;
            vector<Mat> bgr;
            Rect rectCore = r->boundingRect();
            Rect rectSurround;
            rectSurround.x = (int)(rectCore.x - width / 5 > 0 ? rectCore.x - width / 5 : 1);
            rectSurround.y = (int)(rectCore.y - height / 5 > 0 ? rectCore.y - height / 5 : 1);
            rectSurround.width = (int)(width * 1.4);
            rectSurround.height = (int)(height * 1.2);
            Mat roi = Mat((*img)(rectSurround));
            if (roi.data == NULL)
                return -1;
            split(roi, bgr);
            Mat red = bgr[2];
            Mat blue = bgr[0];

            for (size_t i = 0; i < red.rows; i++)
            {
                uchar *data_r = red.ptr<uchar>(i);
                uchar *data_b = blue.ptr<uchar>(i);
                for (size_t j = 0; j < red.cols; j++)
                {
                    if (!rectCore.contains(Point2d(j, i)))
                    {
                        uchar red_v = data_r[j];
                        uchar blue_v = data_b[j];
                        pixNUM++;
                        if (red_v > blue_v)
                        {
                            red_pix_c++;
                        }
                        if (blue_v > red_v)
                        {
                            blue_pix_c++;
                        }
                    }
                }
            }
            //  cout << "\tcolor:" << (double)red_pix_c / (double)blue_pix_c << "\n";
            return ((double)red_pix_c / (double)blue_pix_c) > 2.100;
        }
        catch (cv::Exception e)
        {
        }
        catch (exception ee)
        {
        }
    }
    else
        return -1;
}

myRect *chooseTargetInMyRectVec(vector<myRect> *myr)
{
    myRect *ace = NULL;
    size_t mix_score_avaliable = 60;
    int high_score = 0;
    for (size_t n = 0; n < myr->size(); ++n)
    {
        size_t score = myr->at(n).score;
        if (score >= high_score)
        {
            high_score = score;
            ace = &(myr->at(n));
        }
    }
    return ace;
}

double get_point_angle(Point2d pointO, Point2d pointA)
{
    double angle = 0;
    CvPoint point;
    double temp;
    point = Point2d((pointA.x - pointO.x), (pointA.y - pointO.y));
    if ((0 == point.x) && (0 == point.y))
    {
        return 0;
    }

    if (0 == point.x)
    {
        angle = 90;
        return angle;
    }

    if (0 == point.y)
    {
        angle = 0;
        return angle;
    }
    temp = fabs(float(point.y) / float(point.x));
    temp = atan(temp);
    angle = temp * 180 / CV_PI;
    return angle;
}

double roatedRect_ang(vector<Point> points)
{
    return getOrientation(points);
}

size_t computeArmaScore(myRect *mr)
{
#define NO_LOG_SCORE
    size_t score = 0;
    double ang1 = mr->left_ang;
    double ang2 = mr->right_ang;

    /*角度差值 越小越好*/
    double AngDiffer = abs(ang1 - ang2);
    if (AngDiffer > 0 && AngDiffer < 5.0)
    {
        score += 40;
    }
    else if (AngDiffer >= 5.0 && AngDiffer < 9.0)
    {
        score += 25;
    }
    else if (AngDiffer >= 9.0 && AngDiffer < 15.0)
    {
        score += 10;
    }

    /*偏离垂直方向角度绝对值 越小越好*/
    double DiffBeCz = (abs(90 - ang1) + abs(90 - ang2)) / 2;
    if (DiffBeCz > 0 && DiffBeCz < 7.5)
    {
        score += 20;
    }
    else if (DiffBeCz >= 7.5 && DiffBeCz < 15)
    {
        score += 10;
    }
    else if (DiffBeCz >= 15 && DiffBeCz < 30)
    {
        score += 5;
    }

    /*左右灯柱中心点的连线与水平线之间的角度值 越小越好*/
    int x1 = getOrientationX(mr->leftPoints);  //(int)mr->left.center.x;
    int y1 = getOrientationY(mr->leftPoints);  //(int)mr->left.center.y;
    int x2 = getOrientationX(mr->rightPoints); //(int)mr->right.center.x;
    int y2 = getOrientationY(mr->rightPoints); //(int)mr->right.center.y;
    double line_ang = get_point_angle(Point2f(x1, y1), Point2f(x2, y2));
    if (line_ang > 0 && line_ang < 10.0)
    {
        score += 20;
    }
    else if (line_ang >= 10.0 && line_ang < 15)
    {
        score += 10;
    }
    else if (line_ang >= 15 && line_ang < 30)
    {
        score += 5;
    }
    else if (line_ang >= 30.0)
    {
        score += 1;
    }

    /*左右灯柱形态差距 越小越好*/
    float w1 = mr->left.size.width < mr->left.size.height ? mr->left.size.width : mr->left.size.height;
    float h1 = mr->left.size.width > mr->left.size.height ? mr->left.size.width : mr->left.size.height;
    float w2 = mr->right.size.width < mr->right.size.height ? mr->right.size.width : mr->right.size.height;
    float h2 = mr->right.size.width > mr->right.size.height ? mr->right.size.width : mr->right.size.height;
    double XT = abs((h1 / w1) - (h2 / w2));
    if (XT > 0 && XT < 0.7)
    {
        score += 20;
    }
    else if (XT >= 0.7 && XT < 1.0)
    {
        score += 10;
    }
    else if (XT >= 1.0 && XT < 0.2)
    {
        score += 5;
    }
    mr->set(score);
#ifndef NO_LOG_SCORE
    cout << "垂直方向 " << DiffBeCz << "\n";
    cout << "角度偏差 " << AngDiffer << "\n";
    cout << "形态差距" << XT << "\n";
    cout << "水平方向 " << line_ang << "\n";
#endif

    return 0;
}

vector<myRect> make_allArma(vector<RotatedRect> *all, Mat image, vector<vector<Point>> *points)
{

    float basebl = 0.3;
    int MIN_W = 2, MIN_H = 5, MIN_AREA = 20;
    int MAX_ANG = 7.5, MAX_H = 320, MAX_W = 100;
    int CENTER_MAX = 25;
    vector<myRect> mr;
    if (all->size() < 2)
        return mr;
    for (size_t i = 0; i < all->size(); ++i)
    {
        for (size_t j = 0; j < all->size(); ++j)
        {
            RotatedRect *r1 = &all->at(i);
            RotatedRect *r2 = &all->at(j);
            vector<Point> point1 = points->at(i);
            vector<Point> point2 = points->at(j);
            float r1_width = r1->size.width < r1->size.height ? r1->size.width : r1->size.height;
            float r1_heigh = r1->size.width > r1->size.height ? r1->size.width : r1->size.height;
            float r2_width = r2->size.width < r2->size.height ? r2->size.width : r2->size.height;
            float r2_heigh = r2->size.width > r2->size.height ? r2->size.width : r2->size.height;
            CENTER_MAX = (r1_heigh + r2_heigh) / 2 / 3 + 8;
            double ang1 = getOrientationY(point1);
            double ang2 = getOrientationY(point2);
            /*the same roatedRect*/

            if (r1->center.x == r2->center.x && r1->center.y == r2->center.y)
            {

#ifdef PRINT
                cout << "0"
                     << "\n";
#endif
                continue;
            }

            if (r1->boundingRect().width > r1->boundingRect().height || r2->boundingRect().width > r2->boundingRect().height)
            {
#ifdef PRINT
                cout << "1"
                     << "\n";
#endif
                continue;
            }
            if (r1_heigh / r2_heigh > 1.3 || r1_heigh / r2_heigh < 0.8)
            {
#ifdef PRINT
                cout << "---"
                     << "\n";
#endif
                continue;
            }

            if (abs(ang1 - ang2) > CENTER_MAX &&
                ang1 > r1->boundingRect().y &&
                ang1 > r2->boundingRect().y &&
                ang2 > r1->boundingRect().y &&
                ang2 > r2->boundingRect().y)
            {
#ifdef PRINT
                cout << "2"
                     << "\n";
#endif
                continue;
            }

            /*width min value*/
            if (r1_width < MIN_W || r2_width < MIN_W)
            {
#ifdef PRINT
                cout << "4"
                     << "\n";
#endif
                continue;
            }

            /*height min value*/
            if (r1_heigh < MIN_H || r2_heigh > MAX_H)
            {
#ifdef PRINT
                cout << "5"
                     << "\n";
#endif
                continue;
            }

            /*width max value*/
            if (r1_width > MAX_W || r2_width > MAX_W)
            {
#ifdef PRINT
                cout << "6"
                     << "\n";
#endif
                continue;
            }
            /*height max value*/
            if (r1_heigh > MAX_H || r2_heigh > MAX_H)
            {
#ifdef PRINT
                cout << "7"
                     << "\n";
#endif
                continue;
            }
            if (abs(roatedRect_ang(point1) - roatedRect_ang(point2)) > MAX_ANG)
            {
#ifdef PRINT
                cout << "++"
                     << "\n";
#endif
                continue;
            }
            if (!patchArmaFom_model_vector(r1, r2, image))
            {
#ifdef PRINT
                cout << "9"
                     << "\n";
#endif
                continue;
            }
            mr.push_back(*new myRect(*r1, *r2, i, j, ang1, ang2, point1, point2));
        }
    }
    return mr;
}

bool patchArmaFom_model_vector(RotatedRect *left, RotatedRect *right, Mat binayImag)
{
#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

    RotatedRect LEFT = left->center.x < right->center.x ? *left : *right;
    RotatedRect RIGHT = left->center.x > right->center.x ? *left : *right;
    const Point &pl = LEFT.center, &pr = RIGHT.center;
    Point2f center = (pl + pr) / 2.0;
    cv::Size2f wh_l = LEFT.size;
    cv::Size2f wh_r = RIGHT.size;
    float width = POINT_DIST(pl, pr); // - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height > wh_l.width ? wh_l.height : wh_l.width, wh_r.height > wh_r.width ? wh_r.height : wh_r.width);
    float angle = std::atan2(RIGHT.center.y - LEFT.center.y, RIGHT.center.x - LEFT.center.x);
    RotatedRect r(center, Size2f(width, height), angle * 180 / CV_PI);
    Point2f po[4];
    r.points(po);
    line(binayImag, po[0], po[1], Scalar::all(255));
    line(binayImag, po[1], po[2], Scalar::all(255));
    line(binayImag, po[2], po[3], Scalar::all(255));
    line(binayImag, po[0], po[3], Scalar::all(255));
    float bl = abs(width / height);
    // printf("%f\n", bl);
    // imshow("ddd", binayImag);
    return abs(bl - 2.5) < 1; //|| abs(bl - 4.6) < 0.31;
}

double getOrientation(vector<Point> &pts)
{
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    Point pos = Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
    }
    return atan2(eigen_vecs[0].y, eigen_vecs[0].x) * 180 / CV_PI;
}

int getOrientationY(vector<Point> pts)
{
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    Point pos = Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));
    return pos.y;
}

int getOrientationX(vector<Point> pts)
{
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    Point pos = Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));
    return pos.x;
}