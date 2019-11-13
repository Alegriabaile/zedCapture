//
// Created by b501 on 2019/11/12.
//

#ifndef ZEDCAPTURE_CREATEPATHORFILE_H
#define ZEDCAPTURE_CREATEPATHORFILE_H

////////////////////////// file ////////////////////////////////////////////////////////////

///* Values for the second argument to access.
//   These may be OR'd together.  */
//#define	R_OK	4		/* Test for read permission.  */
//#define	W_OK	2		/* Test for write permission.  */
//#define	X_OK	1		/* Test for execute permission.  */
//#define	F_OK	0		/* Test for existence.  */
//
///* Test for access to NAME using the real UID and real GID.  */
//extern int access (const char *__name, int __type) __THROW __nonnull ((1));



//////////////////////// time //////////////////////////////////////////////////////////////
//
//char *asctime(const struct tm* timeptr);
//将结构中的信息转换为真实世界的时间，以字符串的形式显示
//
//
//
//char *ctime(const time_t* timep);
//将timep转换为真是世界的时间，以字符串显示，它和asctime不同就在于传入的参数形式不一样
//
//
//
//double difftime(time_t time1, time_t time2);
//返回两个时间相差的秒数
//
//
//
//int gettimeofday(struct timeval* tv,struct timezone* tz);
//返回当前距离1970年的秒数和微妙数，后面的tz是时区，一般不用
//
//
//
//struct tm* gmtime(const time_t* timep);
//将time_t表示的时间转换为没有经过时区转换的UTC时间，是一个struct tm结构指针
//
//
//
//struct tm* localtime(const time_t* timep);
//和gmtime类似，但是它是经过时区转换的时间。
//
//
//
//time_t mktime(struct tm* timeptr);
//将struct tm 结构的时间转换为从1970年至今的秒数
//
//
//
//        time_t time(time_t* t);
//取得从1970年1月1日至今的秒数。
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

template <typename T>
class MyVector6
{
private:
    static const int nbElem = 6;
    unsigned int state;
public:
    MyVector6()
    {
        for(int i=0; i<nbElem; ++i)
            v[i] = T(0);
        state = 0;
    }
    /// @cond
    union {

        struct {
            T x, y, z, nx, ny, nz;
        };

        struct {
            T  px, py, pz, r, g, b;
        };

        struct {
            T rx, ry, rz, tx, ty, tz;
        };
        T v[nbElem];
    };

    T &operator[](int i) {
        return this->v[i];
    }

    void setState(unsigned int x)
    {
        state = x;
    }
    unsigned int getState()
    {
        return state;
    }

};
typedef MyVector6<float> float6pose;


class ZedIoUtils
{
private:
    std::string m_dir;
    std::string m_imageDir, m_depthDir, m_paramDir, m_imageDirR;
    unsigned int count;

    void tm_to_string(struct tm* ptm, std::string& res)
    {
        if(ptm == NULL)
            return;

        using namespace std;
        res.clear();

        int ymdhms[6] = {0};
        ymdhms[0] = 1900 + ptm->tm_year;
        ymdhms[1] = 1 + ptm->tm_mon;
        ymdhms[2] = ptm->tm_mday;
        ymdhms[3] = ptm->tm_hour;
        ymdhms[4] = ptm->tm_min;
        ymdhms[5] = ptm->tm_sec;

        for(int i = 0; i < 6; ++i)
        {
            if(ymdhms[i] < 10)
                res.append("0");
            res.append(to_string(ymdhms[i]));
        }
    }

    void SaveExtrinsics(const std::string & paraFileName, float6pose & pose)
    {
        std::ofstream outputfile;
        outputfile.open (paraFileName);

        if (outputfile.is_open())
        {
            for (int i=0; i<5; ++i)
                outputfile << pose[i]<<" ";

            outputfile<<pose[5];
        } else
        {
            std::cout<<"ZedIoUtils::SaveExtrinsics(): open file failed"<<std::endl;
            return;
        }

        outputfile.close();

    }


public:
    ZedIoUtils():count(1000)
    {
        using namespace std;
        int state = 0;
        __mode_t fullAccess = 7 | 7<<3 | 7<<6;

        std::string zedDataDir("zedDataDir");
        if( 0 == access(zedDataDir.c_str(), F_OK))
            cout<<"Path "<<zedDataDir<<" exists"<<endl;
        else
        {
            state = mkdir(zedDataDir.c_str(), fullAccess);
            if(state == 0)
                cout<<zedDataDir<<" : mkdir succeed!"<<endl;
            else
                cout<<zedDataDir<<" : mkdir failed!"<<endl;
        }

        time_t timep;
        struct tm *p;

        time(&timep); /*获得time_t结构的时间，UTC时间*/
        p = localtime(&timep); /*转换为struct tm结构的UTC时间*/

        std::string res;
        tm_to_string(p, res);

        m_dir.clear();
        m_dir.append(zedDataDir).append("/").append(res);

        state = mkdir(m_dir.c_str(), fullAccess);
        m_imageDir = m_dir + "/Images";
        m_depthDir = m_dir + "/Depths";
        m_paramDir = m_dir + "/Params";
        m_imageDirR = m_dir + "/Images_R";

        state |= mkdir(m_imageDir.c_str(), fullAccess);
        state |= mkdir(m_depthDir.c_str(), fullAccess);
        state |= mkdir(m_paramDir.c_str(), fullAccess);
        state |= mkdir(m_imageDirR.c_str(), fullAccess);

        if(state != 0)
            cout<<"ZedIoUtils::ZedIoUtils():    mkdir() failed!!!"<<endl;
    }
    ~ZedIoUtils(){}

    void saveFrame(cv::Mat &imageL, cv::Mat &depth, cv::Mat &imageR, float6pose pose = float6pose())
    {
        using std::string;
        using std::to_string;
        using cv::imwrite;
        using cv::Mat;
        Mat newL = imageL;
        Mat newR = imageR;
        Mat newDepth = depth;
        if(imageL.type() != CV_8UC3)
            imageL.convertTo(newL, CV_8UC3);
        if(imageR.type() != CV_8UC3)
            imageR.convertTo(newR, CV_8UC3);
        if(depth.type() != CV_16UC1)
            depth.convertTo(newDepth, CV_16UC1);

        string name;
        name = m_imageDir+"/"+to_string(count)+".jpg";
        imwrite( name, newL);
        name = m_imageDirR + "/" + to_string(count) + ".jpg";
        imwrite( name, newR);

        //
        cv::Mat mask = newDepth > 20000;
        newDepth.setTo(0, mask);

        double minVal, maxVal;
        cv::minMaxLoc(newDepth, &minVal, &maxVal);
        std::cout<<"minVal, maxVal:  "<<minVal<<", "<<maxVal<<std::endl;

//        cv::imshow("debug_depth", newDepth);
        name = m_depthDir + "/" + to_string(count) + ".png";
        imwrite( name, newDepth);

        if(pose.getState() == 1)
        {
            string paraName;
            paraName.append(m_paramDir).append("/").append(to_string(count)).append(".txt");
            SaveExtrinsics(paraName, pose);
        }

        ++count;
    }
};

#endif //ZEDCAPTURE_CREATEPATHORFILE_H
