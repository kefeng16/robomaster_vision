
#ifndef IMGPRDO
#define IMGPRDO
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <cmath>
#define color size_t
#ifndef RED
#define RED 1
#endif
#ifndef BLUE
#define BLUE 0
#endif
#define NOPRINT
#define NODEBUG
using namespace std;
using namespace cv;

typedef struct left_right_rect
{
    RotatedRect left;
    size_t left_id;
    RotatedRect right;
    size_t right_id;
    unsigned int score;
    double left_ang;
    double right_ang;
    vector<Point> leftPoints;
    vector<Point> rightPoints;

    void set(unsigned int score)
    {
        this->score = score;
    }

    left_right_rect() {}

    left_right_rect(RotatedRect r1, RotatedRect r2, size_t lid, size_t rid, double ang1, double ang2, vector<Point> leftPoints, vector<Point> rightPoints)
    {
        this->left = r1;
        this->right = r2;
        this->left_id = lid;
        this->right_id = rid;
        this->left_ang = ang1;
        this->right_ang = ang2;
        this->leftPoints = leftPoints;
        this->rightPoints = rightPoints;
    }
} myRect;

Mat produceImage(Mat *real, Rect *rect);

color checkOuter(RotatedRect *r, Mat *img);

Mat findContourInEnemyColor(Mat *frame, color col, std::vector<RotatedRect> *rotateRects, std::vector<std::vector<Point>> *points);

double get_points_ang(std::vector<Point> points);

myRect *chooseTargetInMyRectVec(std::vector<myRect> *myr);

size_t computeArmaScore(myRect *mr);

std::vector<myRect> make_allArma(std::vector<RotatedRect> *all, Mat image, std::vector<std::vector<Point>> *points);

bool patchArmaFom_model_vector(RotatedRect *left, RotatedRect *right, Mat binayImag);

double getOrientation(std::vector<Point> &pts);

int getOrientationY(std::vector<Point> pts);

int getOrientationX(vector<Point> pts);

void ArmorDetection(Mat frame, int fd, color col);

#endif