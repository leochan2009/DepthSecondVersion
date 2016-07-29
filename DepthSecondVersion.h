//------------------------------------------------------------------------------
// <copyright file="DepthSecondVersion.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
//#include "DepthImageServer.cxx"
#include "DepthImageServerX264.cpp"
using namespace DepthImageServerX264;
int CheckNeighbors(uint8_t *RGBFrame, int checkIndex, int nWidth, int nHeight)
{
  if (RGBFrame[3* checkIndex]==0 && RGBFrame[3 * checkIndex+1]==0 && RGBFrame[3 * checkIndex+2]==0)
  { 
    return checkIndex;
  }
  else if (RGBFrame[3 * (checkIndex + 1)] == 0 && RGBFrame[3 * (checkIndex + 1) + 1] == 0 && RGBFrame[3 * (checkIndex + 1) + 2] == 0)
  {
    return (checkIndex+1)>=(nHeight*nWidth)?(nHeight*nWidth-1):(checkIndex + 1);
  }
  else if (RGBFrame[3 * (checkIndex - 1)] == 0 && RGBFrame[3 * (checkIndex - 1) + 1] == 0 && RGBFrame[3 * (checkIndex - 1) + 2] == 0)
  {
    return (checkIndex - 1) < 0 ? 0 : (checkIndex - 1);
  }
  else if (RGBFrame[3 * (checkIndex + nWidth)] == 0 && RGBFrame[3 * (checkIndex + nWidth) + 1] == 0 && RGBFrame[3 * (checkIndex + nWidth) + 2] == 0)
  {
    return (checkIndex + nWidth) >= (nHeight*nWidth) ? (nHeight*nWidth - 1) : (checkIndex + nWidth);
  }
  else if (RGBFrame[3 * (checkIndex - nWidth)] == 0 && RGBFrame[3 * (checkIndex - nWidth) + 1] == 0 && RGBFrame[3 * (checkIndex - nWidth) + 2] == 0)
  {
    return (checkIndex - nWidth) < 0 ? 0 : (checkIndex - nWidth);
  }
}

class CDepthSecondVersion
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;
    static const int        cColorWidth = 1920;
    static const int        cColorHeight = 1080;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CDepthSecondVersion();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CDepthSecondVersion();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;
    bool                    m_bSaveScreenshot;

    // Current Kinect
    IKinectSensor*          m_pKinectSensor;

    // Coordinate Mapper
    ICoordinateMapper*      m_pCoordinateMapper;

    // Depth reader
    IDepthFrameReader*      m_pDepthFrameReader;

    // Color reader
    IColorFrameReader*      m_pColorFrameReader;

    IMultiSourceFrameReader* m_pMultiSourceReader;

    // Direct2D
    ImageRenderer*          m_pDrawDepth;
    ImageRenderer*          m_pDrawColor;
    ID2D1Factory*           m_pD2DFactory;
    RGBQUAD*                m_pDepthRGBX;
    RGBQUAD*                m_pColorRGBX;

    DepthSpacePoint *m_pDepthCoordinates;
    BufferedData m_pDepthFrameYUV420;
    BufferedData m_pDepthIndexYUV420;
    SFrameBSInfo info;
    x264_picture_t picDepthFrame;
    x264_picture_t picDepthIndex;
    BufferedData m_pColorYUV444;
    SFrameBSInfo infoColor;
    x264_picture_t picColor;

    igtl::MultiThreader::Pointer threaderServer;
    igtl::MutexLock::Pointer glockServer;
    DepthImageServerX264::ThreadDataServer td_Server;
    igtl::SimpleMutexLock *localMutex;

    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();

    /// <summary>
    /// Handle new depth data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// <param name="nMinDepth">minimum reliable depth</param>
    /// <param name="nMaxDepth">maximum reliable depth</param>
    /// </summary>
    void                    ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

    /// <summary>
    /// Handle new depth data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// </summary>
    void                    ProcessColor(INT64 nTime, UINT16*pBuffer, RGBQUAD* pBufferColor, int nWidth, int nHeight, int nWidthColor, int nHeightColor);


    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);
};

