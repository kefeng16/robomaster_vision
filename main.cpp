#include "imgprdo.hpp"
#include "angslu.hpp"
#include "serial.h"
#define USESERPORT
#define DEVICE "/dev/ttyUSB0"
// cv::Mat _binary_template;       // armor template binary image
// cv::Mat _binary_template_small; // small armor template binay image
// cv::Mat _src;                   // source image
// cv::Mat _g;                     // green component of source image
// cv::Mat _ec;                    // enemy color
// cv::Mat _max_color;             // binary image of sub between blue and red component
// cv::Size _size;
// cv::RotatedRect _res_last; // last detect result
// cv::Rect _dect_rect;       // detect roi of original image
// bool makeRectSafe(cv::Rect &rect, cv::Size size)
// {
//     if (rect.x < 0)
//         rect.x = 0;
//     if (rect.x + rect.width > size.width)
//         rect.width = size.width - rect.x;
//     if (rect.y < 0)
//         rect.y = 0;
//     if (rect.y + rect.height > size.height)
//         rect.height = size.height - rect.y;
//     if (rect.width <= 0 || rect.height <= 0)
//         return false;
//     return true;
// }
// void setImage(const cv::Mat &src)
// {
//     _size = src.size();
//     const cv::Point &last_result = _res_last.center;
//     if (last_result.x == 0 || last_result.y == 0)
//     {
//         _src = src;
//         _dect_rect = Rect(0, 0, src.cols, src.rows);
//     }
//     else
//     {
//         Rect rect = _res_last.boundingRect();
//         int max_half_w = _para.max_light_delta_h * 1.3;
//         int max_half_h = 300;
//         double scale = src.rows == 480 ? 1.8 : 2.5;

//         int exp_half_w = min(max_half_w / 2, int(rect.width * scale));
//         int exp_half_h = min(max_half_h / 2, int(rect.height * scale));

//         int w = std::min(max_half_w, exp_half_w);
//         int h = std::min(max_half_h, exp_half_h);
//         Point center = last_result;
//         int x = std::max(center.x - w, 0);
//         int y = std::max(center.y - h, 0);
//         Point lu = Point(x, y);
//         x = std::min(center.x + w, src.cols);
//         y = std::min(center.y + h, src.rows);
//         Point rd = Point(x, y);
//         _dect_rect = Rect(lu, rd);
//         if (makeRectSafe(_dect_rect, src.size()) == false)
//         {
//             _res_last = cv::RotatedRect();
//             _dect_rect = Rect(0, 0, src.cols, src.rows);
//             _src = src;
//         }
//         else
//             src(_dect_rect).copyTo(_src);
//     }

//     int total_pixel = _src.cols * _src.rows;
//     const uchar *ptr_src = _src.data;
//     const uchar *ptr_src_end = _src.data + total_pixel * 3;

//     _g.create(_src.size(), CV_8UC1);
//     _ec.create(_src.size(), CV_8UC1);
//     _max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));
//     uchar *ptr_g = _g.data, *ptr_ec = _ec.data, *ptr_max_color = _max_color.data;
//     if (_para.enemy_color == RED)
//     {
//         for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)
//         {
//             uchar b = *ptr_src;
//             uchar g = *(++ptr_src);
//             uchar r = *(++ptr_src);
//             *ptr_g = g;
//             *ptr_ec = r;
//             //*ptr_g = b;
//             if (r > _para.min_light_gray)
//                 *ptr_max_color = 255;
//             //            if (r - b > _para.br_threshold && r >= g)
//             //                *ptr_max_color = 255;
//         }
//     }
//     else
//     {
//         for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)
//         {
//             uchar b = *ptr_src;
//             uchar g = *(++ptr_src);
//             uchar r = *(++ptr_src);
//             *ptr_g = g;
//             *ptr_ec = b;
//             //*ptr_g = r;
//             if (b > _para.min_light_gray)
//                 *ptr_max_color = 255;
//             //            if (b - r > _para.br_threshold && b >= g)
//             //                *ptr_max_color = 255;
//         }
//     }

// #ifdef SHOW_DEBUG_IMG
//     cv::imshow("g", _g);
//     cv::imshow("_max_color", _max_color);
// #endif
// }
int main(int argc, char *argv[])
{
    int fd = -1;
#ifdef USESERPORT
    fd = openPort(DEVICE);
    if (fd != -1)
    {
        configurePort(fd);
        printf("open port /dev/ttyUSB0 done.\n");
    }
    else
        printf("open_port fail: Unable to open port. \n");
        // double buff[3] = {0, 0, 0};
        // while (1)
        // {
        //     sendXYZ(fd, buff);
        // }
#endif
    VideoCapture cap(1);
    cout << cap.set(CAP_PROP_FRAME_WIDTH, 640) << "\n";
    cout << cap.set(CAP_PROP_FRAME_HEIGHT, 480) << "\n";
    cout << cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G')) << endl;
    cout << cap.set(CAP_PROP_FPS, 120) << "\n";
    // while(1);

    Mat frame;
    clock_t start, end;
    while (cap.read(frame))
    {
        //  * para frame : current frame
        //  * para fd : file descriptor to port
        //  * para col : color
        start = clock();
        ArmorDetection(frame, fd, BLUE);
        end = clock();
        double fps = 1 / ((double)(end - start) / CLOCKS_PER_SEC);
        putText(frame, "fps: " + to_string(fps), Point(10, 20), 1, 1.2, Scalar(0, 0, 255));
        imshow("frame", frame);
        waitKey(1);
    }
    return 0;
}
