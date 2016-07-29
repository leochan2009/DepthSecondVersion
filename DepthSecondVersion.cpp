//------------------------------------------------------------------------------
// <copyright file="DepthSecondVersion.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#define NOMINMAX

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthSecondVersion.h"

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CDepthSecondVersion application;
    application.Run(hInstance, nShowCmd);
}


void Bitmap2Yuv444p_calc2(uint8_t *destination, uint8_t *rgb, size_t width, size_t height)
{
  size_t image_size = width * height;
  size_t upos = image_size;
  size_t vpos = image_size*2;
  size_t i = 0;

  for (size_t line = 0; line < height; ++line)
  {
    for (size_t x = 0; x < width; x ++)
    {
      uint8_t r = rgb[3 * i];
      uint8_t g = rgb[3 * i + 1];
      uint8_t b = rgb[3 * i + 2];

      destination[i] = ((66 * r + 129 * g + 25 * b) >> 8) + 16;

      destination[upos+i] = ((-38 * r + -74 * g + 112 * b) >> 8) + 128;
      destination[vpos+i] = ((112 * r + -94 * g + -18 * b) >> 8) + 128;
      i++;
    }
  }
}

/// <summary>
/// Constructor
/// </summary>
CDepthSecondVersion::CDepthSecondVersion() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_bSaveScreenshot(false),
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pDrawDepth(NULL),
    m_pDrawColor(NULL),
    m_pDepthRGBX(NULL),
    m_pColorRGBX(NULL),
    m_pMultiSourceReader(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
    this->localMutex = new igtl::SimpleMutexLock;
    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
    // create heap storage for the coorinate mapping from color to depth
    m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];
    x264_picture_alloc(&picDepthFrame, X264_CSP_I420, cDepthWidth, cDepthHeight);
    x264_picture_alloc(&picDepthIndex, X264_CSP_I420, cDepthWidth, cDepthHeight);
    x264_picture_alloc(&picColor, X264_CSP_RGB, cColorWidth, cColorHeight);
    int frameSize = cDepthWidth*cDepthHeight;
    m_pDepthFrameYUV420.SetLength(frameSize);
    picDepthFrame.img.i_plane = 1;
    picDepthFrame.img.plane[0] = m_pDepthFrameYUV420.data();
    picDepthFrame.img.i_stride[0] = cDepthWidth;
    m_pDepthIndexYUV420.SetLength(frameSize);
    picDepthIndex.img.i_plane = 1;
    picDepthIndex.img.plane[0] = m_pDepthIndexYUV420.data();
    picDepthIndex.img.i_stride[0] = cDepthWidth;
    m_pColorYUV444.SetLength(cDepthWidth* cDepthHeight * 3 );
    picColor.img.i_plane = 3;
    picColor.img.i_stride[0] = picColor.img.i_stride[1] = picColor.img.i_stride[2] = cColorWidth;
    picColor.img.plane[0] = m_pColorYUV444.data();
    picColor.img.plane[1] = picColor.img.plane[0] + cDepthWidth * cDepthHeight;
    picColor.img.plane[2] = picColor.img.plane[1] + cDepthWidth * cDepthHeight;
    
    // Initial the openigtlink server
    threaderServer = igtl::MultiThreader::New();
    glockServer = igtl::MutexLock::New();
    td_Server.portNum = 18944;
    td_Server.stop = 1;
    td_Server.pic_DepthFrame = picDepthFrame;
    td_Server.pic_DepthIndex = picDepthIndex;
    td_Server.pic_Color = picColor;
    td_Server.transmissionFinished = true;
    td_Server.conditionVar = igtl::ConditionVariable::New();
    threaderServer->SpawnThread((igtl::ThreadFunctionType) &ServerControl, &td_Server);
    
}
  

/// <summary>
/// Destructor
/// </summary>
CDepthSecondVersion::~CDepthSecondVersion()
{
    // clean up Direct2D renderer
    if (m_pDrawDepth)
    {
        delete m_pDrawDepth;
        m_pDrawDepth = NULL;
    }

    if (m_pDrawColor)
    {
      delete m_pDrawColor;
      m_pDrawColor = NULL;
    }


    if (m_pDepthRGBX)
    {
      delete[] m_pDepthRGBX;
      m_pDepthRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

    // done with depth frame reader
    SafeRelease(m_pColorFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthSecondVersion::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"DepthSecondVersionAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CDepthSecondVersion::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);
    // Main message loop


    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CDepthSecondVersion::Update()
{
    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;
    IColorFrame* pColorFrame = NULL;
    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

        IFrameDescription* pFrameDescriptionColor = NULL;
        int nWidthColor = 0;
        int nHeightColor = 0;
        UINT nBufferSizeColor = 0;
        RGBQUAD *pBufferColor = NULL;

        hr = pDepthFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        }

        if (SUCCEEDED(hr))
        {
			    // In order to see the full range of depth (including the less reliable far field depth)
			    // we are setting nDepthMaxDistance to the extreme potential depth threshold
			    //nDepthMaxDistance = USHRT_MAX;

			    // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
          hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
        }

        if (SUCCEEDED(hr))
        {
          //nDepthMaxDistance = nDepthMinReliableDistance + 255;
          ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }
        SafeRelease(pFrameDescription);

        hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
        if (SUCCEEDED(hr))
        {
          hr = pColorFrame->get_FrameDescription(&pFrameDescriptionColor);
        }

        if (SUCCEEDED(hr))
        {
          hr = pFrameDescriptionColor->get_Width(&nWidthColor);
        }

        if (SUCCEEDED(hr))
        {
          hr = pFrameDescriptionColor->get_Height(&nHeightColor);
        }

        ColorImageFormat imageFormat = ColorImageFormat_None;
        if (SUCCEEDED(hr))
        {
          hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr))
        {
          if (imageFormat == ColorImageFormat_Bgra)
          {
            hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSizeColor, reinterpret_cast<BYTE**>(&pBufferColor));
          }
          else if (m_pColorRGBX)
          {
            pBufferColor = m_pColorRGBX;
            nBufferSizeColor = cColorWidth * cColorHeight * sizeof(RGBQUAD);
            hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSizeColor, reinterpret_cast<BYTE*>(pBufferColor), ColorImageFormat_Bgra);
          }
          else
          {
            hr = E_FAIL;
          }
        }
        if (SUCCEEDED(hr))
        {
          ProcessColor(nTime, pBuffer, pBufferColor,nWidth,nHeight, nWidthColor, nHeightColor);
        }
        if (Synchonize)
        {
          this->localMutex->Lock();
          this->td_Server.transmissionFinished = false;
          while (!this->td_Server.transmissionFinished)
            this->td_Server.conditionVar->Wait(this->localMutex);
          this->localMutex->Unlock();
        }
        SafeRelease(pFrameDescriptionColor);
    }
    SafeRelease(pDepthFrame);
    SafeRelease(pColorFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthSecondVersion::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CDepthSecondVersion* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CDepthSecondVersion*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CDepthSecondVersion*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>

LRESULT CALLBACK CDepthSecondVersion::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawDepth = new ImageRenderer();
            m_pDrawColor = new ImageRenderer();
            HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cDepthWidth, cDepthHeight, cDepthWidth * sizeof(RGBQUAD)); 
            hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOCOLORVIEW), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
            LPTSTR lpString = L"18944";
            SetDlgItemText(m_hWnd, IDC_EDIT1, lpString);
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
			      // If it was for the check box, start the server
			      if (IDC_CHECK3 == LOWORD(wParam))
			      {
              LPTSTR lpString= new TCHAR[5];
              GetDlgItemText(m_hWnd, IDC_EDIT1, lpString, 5);
              td_Server.portNum = atoi((const char*)lpString);
              td_Server.stop = (bool) HIWORD(wParam);
			      }
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CDepthSecondVersion::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;
        IColorFrameSource* pColorFrameSource = NULL;
        //m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth, &m_pMultiSourceReader);
        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
          m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }


        SafeRelease(pDepthFrameSource);
        SafeRelease(pColorFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void CDepthSecondVersion::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    if (m_hWnd)
    {
        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }

    // Make sure we've received valid data
    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {
        RGBQUAD* pRGBX = m_pDepthRGBX;

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
        uint8_t* pDepthFrame = m_pDepthFrameYUV420.data();
        uint8_t* pDepthIndex = m_pDepthIndexYUV420.data();
        for(int j = 0; j < nHeight; ++j)
        { 
          for (int i = 0; i<nWidth; ++i)
          {
            USHORT depth = *pBuffer;

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? ((depth- nMinDepth) % 256) : 0);

            pRGBX->rgbRed = intensity;
            pRGBX->rgbGreen = 0;
            pRGBX->rgbBlue = 0;
            if ((depth >= nMinDepth) && (depth <= nMaxDepth))
            {
              *(pDepthFrame + j*cDepthWidth + i) = intensity;
              *(pDepthIndex + j*cDepthWidth + i) = (depth - nMinDepth) / 256 + 1;
            }
            ++pRGBX;
            ++pBuffer;
          }
        }
    }
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void CDepthSecondVersion::ProcessColor(INT64 nTime, UINT16*pBuffer, RGBQUAD* pBufferColor, int nWidth, int nHeight, int nWidthColor, int nHeightColor)
{
  // Make sure we've received valid data
  if (m_pCoordinateMapper &&
    pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight) &&
    pBufferColor && (nWidthColor == cColorWidth) && (nHeightColor == cColorHeight))
  {
    HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(cDepthWidth * cDepthHeight, (UINT16*)pBuffer, nWidthColor * nHeightColor, m_pDepthCoordinates);
    uint8_t RGBFrame[3 * cDepthWidth * cDepthHeight] = { 0 };
    //std::vector<std::vector<uint8_t> >testTemp(cDepthWidth * cDepthHeight, std::vector<uint8_t>(3,0));
    if (SUCCEEDED(hr))
    {
      // loop over output pixels
      RGBQUAD* pSrc = new RGBQUAD();
      int validPoint = 0;
      for (int colorIndex = 0; colorIndex < (nWidthColor*nHeightColor); ++colorIndex)
      {
        // default setting source to copy from the background pixel

        DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

        // Values that are negative infinity means it is an invalid color to depth mapping so we
        // skip processing for this pixel
        if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
        {
          int depthX = static_cast<int>(p.X + 0.5f);
          int depthY = static_cast<int>(p.Y + 0.5f);

         
          if ((depthX >= 0 && depthX < nWidth) && (depthY >= 0 && depthY < nHeight))
          {
            validPoint++;
            int fillIndex = CheckNeighbors(RGBFrame, depthY*nWidth + depthX, nWidth,nHeight);
            // set source for copy to the color pixel
            //pSrc = m_pColorRGBX + colorIndex;
            RGBFrame[3 *fillIndex] = (pBufferColor + colorIndex)->rgbRed;
            RGBFrame[3 *fillIndex + 1] = (pBufferColor + colorIndex)->rgbGreen;
            RGBFrame[3 *fillIndex + 2] = (pBufferColor + colorIndex)->rgbBlue;
            m_pDepthRGBX[fillIndex] = *(pBufferColor + colorIndex);
            pSrc->rgbBlue = 0;
            pSrc->rgbGreen = 0;
            pSrc->rgbRed = 0;
            pBufferColor[colorIndex] = *pSrc;
            
          }
        }
      }
      pSrc = NULL;
      Bitmap2Yuv444p_calc2(m_pColorYUV444.data(), RGBFrame, nWidth, nHeight);
    }
  }
  if (pBufferColor && (nWidthColor == cColorWidth) && (nHeightColor == cColorHeight))
  {
    // Draw the data with Direct2D
    m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBufferColor), cColorWidth * cColorHeight * sizeof(RGBQUAD));
  }
  m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CDepthSecondVersion::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

