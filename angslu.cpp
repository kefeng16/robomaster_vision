#include "angslu.hpp"

// uint16_t CRC_INIT = 0xffff;
// const uint16_t wCRC_Table[256] =
//     {
//         0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
//         0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
//         0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
//         0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
//         0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
//         0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
//         0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
//         0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
//         0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
//         0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
//         0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//         0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
//         0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
//         0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
//         0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
//         0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//         0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
//         0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
//         0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
//         0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
//         0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
//         0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
//         0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
//         0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
//         0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
//         0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
//         0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
//         0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
//         0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
//         0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
//         0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
//         0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};
// /*
// ** Descriptions: CRC16 checksum function   描述：CRC16校验和功能
// ** Input: Data to check,Stream length, initialized checksum  输入：要检查的数据，流长度，已初始化的校验和
// ** Output: CRC checksum   输出：CRC校验和
// */
// uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
// {
//     uint8_t chData;
//     if (pchMessage == NULL)
//     {
//         return 0xFFFF;
//     }
//     while (dwLength--)
//     {
//         chData = *pchMessage++;
//         (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
//                                                       0x00ff];
//     }
//     return wCRC;
// }
// /*
// ** Descriptions: CRC16 Verify function   描述：CRC16验证功能
// ** Input: Data to Verify,Stream length = Data + checksum  输入：要验证的数据，流数据=数据+校验和
// ** Output: True or False (CRC Verify Result)  输出：真或假（CRC校验结果）
// */
// uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
// {
//     uint16_t wExpected = 0;
//     if ((pchMessage == NULL) || (dwLength <= 2))
//     {
//         return 0;
//     }
//     wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
//     return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
//                                                                   pchMessage[dwLength - 1]);
// }
// /*
// ** Descriptions: append CRC16 to the end of data   描述：将CRC16附加到数据结尾
// ** Input: Data to CRC and append,Stream length = Data + checksum   输入：数据到CRC并追加，流长度=数据+校验和
// ** Output: True or False (CRC Verify Result)   输出真或假（CRC）检验结果
// */
// void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
// {
//     uint16_t wCRC = 0;
//     if ((pchMessage == NULL) || (dwLength <= 2))
//     {
//         return;
//     }
//     wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
//     pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
//     pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
// }

/**/

void RectPnPSolver::solvePnP4Points(const std::vector<cv::Point2f> &points2d, cv::Mat &rot, cv::Mat &trans)
{
    if (width_target < 10e-5 || height_target < 10e-5)
    {
        rot = cv::Mat::eye(3, 3, CV_64FC1);
        trans = cv::Mat::zeros(3, 1, CV_64FC1);
        return;
    }
    std::vector<cv::Point3f> point3d;
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));

    cv::Mat r;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);
}

void AngleSolver::setRelationPoseCameraPTZ(const cv::Mat &rot_camera_ptz, const cv::Mat &trans_camera_ptz, double y_offset_barrel_ptz)
{
    rot_camera_ptz.copyTo(rot_camera2ptz);
    trans_camera_ptz.copyTo(trans_camera2ptz);
    offset_y_barrel_ptz = y_offset_barrel_ptz;
}

bool AngleSolver::getAngle(const cv::RotatedRect &rect, double &angle_x, double &angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f &offset)
{

    if (rect.size.height < 1)
        return false;
    // get 3D positon in camera coordinate
    //    double wh_ratio = width_target/height_target;
    //    RotatedRect adj_rect(rect.center, Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
    //    vector<Point2f> target2d;
    //    getTarget2dPoinstion(adj_rect, target2d, offset);

    vector<Point2f> target2d;
    getTarget2dPoinstion(rect, target2d, offset);

    cv::Mat r;
    solvePnP4Points(target2d, r, position_in_camera);
    //position_in_camera.at<double>(2, 0) = 1.4596 * position_in_camera.at<double>(2, 0);  // for camera-2 calibration (unfix center)
    //position_in_camera.at<double>(2, 0) = 1.5348 * position_in_camera.at<double>(2, 0);  // for camera-MH calibration (unfix center)
    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);
    if (position_in_camera.at<double>(2, 0) < min_distance || position_in_camera.at<double>(2, 0) > max_distance)
    {
        cout << "out of range: [" << min_distance << ", " << max_distance << "]\n";
        return false;
    }

    // translate camera coordinate to PTZ coordinate
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
    //    cout << "position_in_camera: " << position_in_camera.t() << endl;
    //    cout << "position_in_ptz: " << position_in_ptz.t() << endl;
    // calculte angles to turn, so that make barrel aim at target
    adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed, current_ptz_angle);

    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos)
{
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat &pos_in_ptz, double &angle_x, double &angle_y, double bullet_speed, double current_ptz_angle)
{
    const double *_xyz = (const double *)pos_in_ptz.data;
    double down_t = 0.0;
    if (bullet_speed > 10e-3)
        down_t = _xyz[2] / 100.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    double alpha = 0.0, theta = 0.0;

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));
    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = -(alpha + theta); // camera coordinate
    }
    else if (xyz[1] < offset_y_barrel_ptz)
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -(alpha - theta); // camera coordinate
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = (theta - alpha); // camera coordinate
    }
    angle_x = atan2(xyz[0], xyz[2]);
    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
    // cout << "angle_x: " << angle_x << "\tangle_y: " << angle_y << "\talpha: " << alpha << "\ttheta: " << theta << endl;
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect &rect, vector<Point2f> &target2d, const cv::Point2f &offset)
{
    Point2f vertices[4];
    rect.points(vertices);
    Point2f lu, ld, ru, rd;
    sort(vertices, vertices + 4, [](const Point2f &p1, const Point2f &p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolverFactory::setTargetSize(double width, double height, TargetType type)
{
    if (type == TARGET_RUNE)
    {
        rune_width = width;
        rune_height = height;
    }
    else if (type == TARGET_ARMOR)
    {
        armor_width = width;
        armor_height = height;
    }
    else if (type == TARGET_SAMLL_ATMOR)
    {
        small_armor_width = width;
        small_armor_height = height;
    }
}

bool AngleSolverFactory::getAngle(const cv::RotatedRect &rect, TargetType type, double &angle_x, double &angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f &offset)
{

    if (slover == NULL)
    {
        std::cout << "slover not set\n";
        return false;
    }

    double width = 0.0, height = 0.0;
    if (type == TARGET_RUNE)
    {
        width = rune_width;
        height = rune_height;
    }
    else if (type == TARGET_ARMOR)
    {
        width = armor_width;
        height = armor_height;
    }
    else if (type == TARGET_SAMLL_ATMOR)
    {
        width = small_armor_width;
        height = small_armor_height;
    }
    cv::RotatedRect rect_rectifid = rect;
    AngleSolverFactory::adjustRect2FixedRatio(rect_rectifid, width / height);
    slover->setTargetSize(width, height);
    return slover->getAngle(rect_rectifid, angle_x, angle_y, bullet_speed, current_ptz_angle, offset);
}
/**/
#define BUFFSIZE 10
void Data_filter(size_t _frame_count, double *Data_buff, double *currentData, double *outputData)
{
    double temp = 0;
    if (_frame_count < BUFFSIZE)
    {
        Data_buff[_frame_count] = *currentData;
    }
    else
    {
        Data_buff[_frame_count % BUFFSIZE - 1] = *currentData;
    }
    for (size_t t = 0; t < BUFFSIZE; ++t)
    {
        temp += Data_buff[t];
    }
  //  *outputData = temp / BUFFSIZE;
}