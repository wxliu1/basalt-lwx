
// created by wxliu on 2023-11-13

#ifndef _WX_SYSTEM_H_
#define _WX_SYSTEM_H_

#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <stdlib.h>
#include <dirent.h>
#include <limits.h>
// #include <string.h>
// #include <stdio.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h> // for dirname() basename()

#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <iostream>

#include <link.h>
#include <dlfcn.h>
#define _GNU_SOURCE

#include <sys/time.h>

#include <string>
using std::string;

#define ACCESS access
#define MKDIR(a) mkdir((a), 0755)
namespace wx {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Quaterniond = Eigen::Quaterniond;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;

enum StateOrder
{
  O_P = 0,
  O_R = 3,
  O_V = 6,
  O_BA = 9,
  O_BG = 12
};


enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};


struct SystemCfg{
    
    string imu_topic {"/imu"};

    bool use_imu{ true };
    double td{ 0.0 };
    //Eigen::Vector3d G{0.0, 0.0, 9.8};
    Vector3d g;

    string log_file_path { "./logs" };
    bool output_log { false };
    
    SystemCfg() = default;
    void ReadSystemCfg();
};

extern SystemCfg sys_cfg_;



struct TFileSystemHelper
{
public:

    /******************************************************************************
     * Recursively create directory if `path` not exists.
     * Return 0 if success.
     *****************************************************************************/
    static int createDirectoryIfNotExists(const char *path)
    {
        struct stat info;
        int statRC = stat(path, &info);
        if( statRC != 0 )
        {
            if (errno == ENOENT)  
            {
                printf("%s not exists, trying to create it \n", path);
                if (! createDirectoryIfNotExists(dirname(strdupa(path))))
                {
                    if (mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
                    {
                        fprintf(stderr, "Failed to create folder %s \n", path);
                        return 1;
                    }
                    else
                        return 0;
                }
                else 
                    return 1;
            } // directory not exists
            if (errno == ENOTDIR) 
            { 
                fprintf(stderr, "%s is not a directory path \n", path);
                return 1; 
            } // something in path prefix is not a dir
            return 1;
        }
        return ( info.st_mode & S_IFDIR ) ? 0 : 1;
    }

    static int CreateDir(char *pDir)
    {
      int i = 0;
      int iRet;
      int iLen;
      char *pszDir;

      if (NULL == pDir)
      {
        return 0;
      }

      // strdup( ) 函数是c语言中常用的一种字符串拷贝库函数，一般和 free( ) 函数成对出现。
      // strdup( )在内部调用了malloc( )为变量分配内存，不需要使用返回的字符串时，需要用free( )释放相应的内存空间，否则会造成内存泄漏。
      pszDir = strdup(pDir);
      iLen = strlen(pszDir);

      // 创建中间目录
      for (i = 0; i < iLen; i++)
      {
        if (pszDir[i] == '\\' || pszDir[i] == '/')
        {
          pszDir[i] = '\0';

          // 如果不存在,创建
          iRet = ACCESS(pszDir, 0);
          if (iRet != 0)
          {
            iRet = MKDIR(pszDir);
            if (iRet != 0)
            {
              return -1;
            }
          }
          // 支持linux,将所有\换成/
          pszDir[i] = '/';
        }
      }

      iRet = MKDIR(pszDir);
      free(pszDir);
      return iRet;
    }
    
    // 最后回传的绝对路径名称结尾没有'/'
    static bool getAbsolutePath(char* absolutePath, const char *execName = nullptr)
    {
        if(execName == nullptr)
        {
            char temp[1024];
            getcwd(temp, 1024); // 获取当前工作目录
            sprintf(absolutePath, "%s", temp);
        }
        else if(*(execName) == '/')
        {
            sprintf(absolutePath, "%s", execName);
        }
        else if(strlen(execName) > 0)
        {
            char temp[1024];
            getcwd(temp, 1024); // 获取当前工作目录
            sprintf(absolutePath, "%s/%s", temp, execName);
            std::cout << "temp=" << temp << " execName=" << execName << std::endl;

            char *ptr = strrchr(absolutePath, '/');
            *ptr = '\0';
            // int nLen = strlen(ptr);
            // if(ptr[nLen - 1] == '.')
            // ptr[nLen - 1] = '\0';
        }
        
        return true;
    }


  static void WriteLog(char* szLog)
  {
    if(!sys_cfg_.output_log)
    {
      return ;
    }

    char szDateTime[100] = { 0 };
    FILE* LogFile;
    LogFile = NULL;
  #ifdef _WIN32
    SYSTEMTIME st;
    GetLocalTime(&st);
    sprintf(szDateTime, "%04d-%02d-%02d %02d:%02d:%02d.%03d: ", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
  #else
    struct timeval tv;
    struct tm *t;
      gettimeofday(&tv, NULL);
      t = localtime(&tv.tv_sec);
      snprintf(szDateTime, 30, "%04d%02d%02d_%02d%02d%02d.%03d ",
      1900+t->tm_year, 1+t->tm_mon, t->tm_mday,t->tm_hour, 
      t->tm_min, t->tm_sec, int(tv.tv_usec / 1000));	
  #endif

    char szLogFileName[260] = { 0 };
    if(1)//(strlen(g_sysIni.m_szProgramName) > 0)
    {
    #ifdef _WIN32	
      sprintf(szLogFileName, "logs/%s_%04d%02d%02d.log", "stereo3", st.wYear, st.wMonth, st.wDay);
    #else	
      // sprintf(szLogFileName, "logs/%s_%04d%02d%02d.log", "stereo3", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday);
      sprintf(szLogFileName, "%s/%s_%04d%02d%02d.log", sys_cfg_.log_file_path.c_str(), "stereo3", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday);
    #endif	
    }

    LogFile = fopen(szLogFileName, "a+");
    if (NULL == LogFile)
    {
      //AfxMessageBox("write log file failure!", MB_OK, 0);
      return;
    }

    
    fwrite(szDateTime, strlen(szDateTime), 1, LogFile);
    fwrite(szLog, strlen(szLog), 1, LogFile);
    fwrite("\n", 1, 1, LogFile);

    fclose(LogFile);
    LogFile = NULL;
  }

};




}


#endif
