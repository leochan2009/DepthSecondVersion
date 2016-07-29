/*=========================================================================

  Program:   Open IGT Link -- Example for Tracking Data Server
  Module:    $RCSfile: $
  Language:  C++
  Date:      $Date: $
  Version:   $Revision: $

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <fstream>
#include "svc/codec_api.h"
#include "svc/codec_def.h"
#include "svc/codec_app_def.h"
#include "utils/BufferedData.h"
#include "utils/FileInputStream.h"
#include "api/sha1.c"
#include "igtl_header.h"
#include "igtl_video.h"
#include "igtlMessageDebugFunction.h"
//#include <pcl/visualization/pcl_visualizer.h>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlVideoMessage.h"
#include "igtlServerSocket.h"
#include "igtlMultiThreader.h"
#include "igtlConditionVariable.h"
#include <math.h>

#define IGTL_IMAGE_HEADER_SIZE          72
bool Synchonize = true;
bool useDemux = true;
int DemuxMethod = 2;
bool useCompressForRGB = false;
namespace DepthImageServer {
  void* ThreadFunction(void* ptr);
  int   SendVideoData(igtl::Socket::Pointer& socket, igtl::VideoMessage::Pointer& videoMsg);

  typedef struct {
    igtl::MutexLock::Pointer glock;
    int   stop;
    int   portNum;
    SFrameBSInfo info;
    SSourcePicture pic;
    SFrameBSInfo info_Color;
    SSourcePicture pic_Color;
    bool transmissionFinished;
    igtl::ConditionVariable::Pointer conditionVar;
  } ThreadDataServer;

  typedef struct {
    int   nloop;
    igtl::MutexLock::Pointer glock;
    igtl::Socket::Pointer socket;
    int   interval;
    int   stop;
    ThreadDataServer* td_Server;
  } ThreadData;

  std::string     polyFile = "";
  struct EncodeFileParam {
    const char* pkcFileName;
    const char* pkcHashStr;
    EUsageType eUsageType;
    int iWidth;
    int iHeight;
    float fFrameRate;
    SliceModeEnum eSliceMode;
    bool bDenoise;
    int  iLayerNum;
    bool bLossless;
    bool bEnableLtr;
    bool bCabac;
    int iTargetBitrate;
    unsigned short iMultipleThreadIdc;
  };

  void EncFileParamToParamExt(EncodeFileParam* pEncFileParam, SEncParamExt* pEnxParamExt) {
    pEnxParamExt->iUsageType = pEncFileParam->eUsageType;
    pEnxParamExt->iPicWidth = pEncFileParam->iWidth;
    pEnxParamExt->iPicHeight = pEncFileParam->iHeight;
    pEnxParamExt->fMaxFrameRate = pEncFileParam->fFrameRate;
    pEnxParamExt->iSpatialLayerNum = pEncFileParam->iLayerNum;

    pEnxParamExt->bEnableDenoise = pEncFileParam->bDenoise;
    pEnxParamExt->bIsLosslessLink = pEncFileParam->bLossless;
    pEnxParamExt->bEnableLongTermReference = pEncFileParam->bEnableLtr;
    pEnxParamExt->iEntropyCodingModeFlag = pEncFileParam->bCabac ? 1 : 0;
    pEnxParamExt->iTargetBitrate = pEncFileParam->iTargetBitrate;
    pEnxParamExt->uiMaxNalSize = 1500;
    //pEnxParamExt->uiIntraPeriod = 1;
    pEnxParamExt->iNumRefFrame = AUTO_REF_PIC_COUNT;
    if (pEncFileParam->eSliceMode != SM_SINGLE_SLICE) //SM_DYN_SLICE don't support multi-thread now
      pEnxParamExt->iMultipleThreadIdc = pEncFileParam->iMultipleThreadIdc; // For Adaptive QP encoding

    for (int i = 0; i < pEnxParamExt->iSpatialLayerNum; i++) {
      pEnxParamExt->sSpatialLayers[i].iVideoWidth = pEncFileParam->iWidth;
      pEnxParamExt->sSpatialLayers[i].iVideoHeight = pEncFileParam->iHeight;
      pEnxParamExt->sSpatialLayers[i].fFrameRate = pEncFileParam->fFrameRate;
      pEnxParamExt->sSpatialLayers[i].iSpatialBitrate = pEncFileParam->iTargetBitrate;
      //pEnxParamExt->sSpatialLayers[i].uiProfileIdc = PRO_UNKNOWN;///< value of profile IDC (PRO_UNKNOWN for auto-detection)
      //pEnxParamExt->sSpatialLayers[i].uiLevelIdc = LEVEL_UNKNOWN;///< value of profile IDC (0 for auto-detection)
      //pEnxParamExt->sSpatialLayers[i].iDLayerQp = 0;///< value of level IDC (0 for auto-detection)
      pEnxParamExt->sSpatialLayers[i].sSliceArgument.uiSliceMode = pEncFileParam->eSliceMode;
      if (pEncFileParam->eSliceMode == SM_SIZELIMITED_SLICE) {
        pEnxParamExt->sSpatialLayers[i].sSliceArgument.uiSliceSizeConstraint = 600;
        pEnxParamExt->bUseLoadBalancing = false;
      }
    }
    pEnxParamExt->iTargetBitrate *= pEnxParamExt->iSpatialLayerNum;
    pEnxParamExt->bEnableFrameSkip = true;
  }

  static EncodeFileParam kFileParamArray =
  {
    "res/Cisco_Absolute_Power_1280x720_30fps.yuv",
    "dfd4666f9b90d5d77647454e2a06d546adac6a7c", CAMERA_VIDEO_REAL_TIME, 512, 424, 1.0f, SM_RASTER_SLICE, false, 1, true, false, true, 5000000, 4
  };
  static EncodeFileParam kFileParamArrayDemux =
  {
    "res/Cisco_Absolute_Power_1280x720_30fps.yuv",
    "dfd4666f9b90d5d77647454e2a06d546adac6a7c", CAMERA_VIDEO_REAL_TIME, 512*2, 424*2, 1.0f, SM_SIZELIMITED_SLICE, false, 1, true, false, true, 20000000, 4
  };

  void* ServerControl(void* ptr)
  {
    //------------------------------------------------------------
    // Parse Arguments
    igtl::MultiThreader::ThreadInfo* info =
      static_cast<igtl::MultiThreader::ThreadInfo*>(ptr);
    ThreadDataServer* tdServer = static_cast<ThreadDataServer*>(info->UserData);
    igtl::MutexLock::Pointer glockServer = tdServer->glock;
    int    port = tdServer->portNum;

    igtl::ServerSocket::Pointer serverSocket;
    serverSocket = igtl::ServerSocket::New();
    int r = serverSocket->CreateServer(port);

    if (r < 0)
    {
      std::cerr << "Cannot create a server socket." << std::endl;
      exit(0);
    }

    igtl::MultiThreader::Pointer threader = igtl::MultiThreader::New();
    igtl::MutexLock::Pointer glock = igtl::MutexLock::New();
    ThreadData td;
    td.td_Server = tdServer;

    while (1)
    {
      if (1/*!tdServer->stop*/)
      {
        //------------------------------------------------------------
        // Waiting for Connection
        int threadID = -1;
        igtl::Socket::Pointer socket;
        socket = serverSocket->WaitForConnection(1000);

        if (socket.IsNotNull()) // if client connected
        {
          std::cerr << "A client is connected." << std::endl;

          // Create a message buffer to receive header
          igtl::MessageHeader::Pointer headerMsg;
          headerMsg = igtl::MessageHeader::New();
          //------------------------------------------------------------
          // loop
          for (;;)
          {
            // Initialize receive buffer
            headerMsg->InitPack();

            // Receive generic header from the socket
            int rs = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
            if (rs == 0)
            {
              if (threadID >= 0)
              {
                td.stop = 1;
                threader->TerminateThread(threadID);
                threadID = -1;
              }
              std::cerr << "Disconnecting the client." << std::endl;
              td.socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
              socket->CloseSocket();
              break;
            }
            if (rs != headerMsg->GetPackSize())
            {
              continue;
            }

            // Deserialize the header
            headerMsg->Unpack();

            // Check data type and receive data body
            if (strcmp(headerMsg->GetDeviceType(), "STT_VIDEO") == 0)
            {
              std::cerr << "Received a STT_VIDEO message." << std::endl;

              igtl::StartVideoDataMessage::Pointer startVideoMsg;
              startVideoMsg = igtl::StartVideoDataMessage::New();
              startVideoMsg->SetMessageHeader(headerMsg);
              startVideoMsg->AllocatePack();

              socket->Receive(startVideoMsg->GetPackBodyPointer(), startVideoMsg->GetPackBodySize());
              int c = startVideoMsg->Unpack(1);
              if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
              {
                td.interval = startVideoMsg->GetResolution();
                td.glock = glock;
                td.socket = socket;
                td.stop = 0;
                threadID = threader->SpawnThread((igtl::ThreadFunctionType) &ThreadFunction, &td);
              }
            }
            else if (strcmp(headerMsg->GetDeviceType(), "STP_VIDEO") == 0)
            {
              socket->Skip(headerMsg->GetBodySizeToRead(), 0);
              std::cerr << "Received a STP_VIDEO message." << std::endl;
              if (threadID >= 0)
              {
                td.stop = 1;
                threader->TerminateThread(threadID);
                threadID = -1;
                std::cerr << "Disconnecting the client." << std::endl;
                td.socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
                socket->CloseSocket();
              }
              break;
            }
            else
            {
              std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
              socket->Skip(headerMsg->GetBodySizeToRead(), 0);
            }
          }
        }
      }
      else
      {
        serverSocket->CloseSocket();
      }
    }

    //------------------------------------------------------------
    // Close connection (The example code never reaches to this section ...
  }

  void* ThreadFunction(void* ptr)
  {
    //------------------------------------------------------------
    // Get thread information
    igtl::MultiThreader::ThreadInfo* info =
      static_cast<igtl::MultiThreader::ThreadInfo*>(ptr);

    //int id      = info->ThreadID;
    //int nThread = info->NumberOfThreads;
    ThreadData* td = static_cast<ThreadData*>(info->UserData);

    //------------------------------------------------------------
    // Get user data
    igtl::MutexLock::Pointer glock = td->glock;
    long interval = td->interval;
    std::cerr << "Interval = " << interval << " (ms)" << std::endl;
    igtl::Socket::Pointer& socket = td->socket;

    //------------------------------------------------------------
    // Allocate TrackingData Message Class
    //
    // NOTE: TrackingDataElement class instances are allocated
    //       before the loop starts to avoid reallocation
    //       in each image transfer.

    ISVCEncoder* encoder_ = NULL;
    ISVCEncoder* encoderColor_ = NULL;
    int rv = WelsCreateSVCEncoder(&encoder_);
    rv += WelsCreateSVCEncoder(&encoderColor_);
    if (rv == 0 && encoder_ != NULL && encoderColor_ != NULL)
    {
      SEncParamExt pEncParamExt;
      memset(&pEncParamExt, 0, sizeof(SEncParamExt));
      SEncParamExt pEncParamExtColor;
      memset(&pEncParamExtColor, 0, sizeof(SEncParamExt));
      //kFileParamArray.iWidth =
      if(useDemux)
      { 
        if (DemuxMethod == 1)
        {
          EncFileParamToParamExt(&kFileParamArrayDemux, &pEncParamExt);
        }
        else if( DemuxMethod == 2)
        {
          EncFileParamToParamExt(&kFileParamArray, &pEncParamExt);
        }
      }
      else
      {
        EncFileParamToParamExt(&kFileParamArray, &pEncParamExt);
      }
      encoder_->InitializeExt(&pEncParamExt);
      int videoFormat = videoFormatI420;
      encoder_->SetOption(ENCODER_OPTION_DATAFORMAT, &videoFormat);

      EncFileParamToParamExt(&kFileParamArray, &pEncParamExtColor);
      encoderColor_->InitializeExt(&pEncParamExtColor);
      encoderColor_->SetOption(ENCODER_OPTION_DATAFORMAT, &videoFormat);
      while (!td->stop)
      {
        int iFrameIdx = 0;
        while ((!td->td_Server->transmissionFinished) && !td->stop)
        {
          td->td_Server->pic.uiTimeStamp = (long long)(iFrameIdx * (1000 / pEncParamExt.fMaxFrameRate));
          iFrameIdx++;
          std::string frameNames[2] = { "DepthFrame", "DepthIndex"};
          for (int iMessage = 0; iMessage < 2; iMessage++)
          {
            igtl::VideoMessage::Pointer videoMsg;
            videoMsg = igtl::VideoMessage::New();
            videoMsg->SetDefaultBodyType("ColoredDepth");
            videoMsg->SetDeviceName(frameNames[iMessage]);
            videoMsg->SetBitStreamSize(pEncParamExt.iPicWidth*pEncParamExt.iPicHeight);
            videoMsg->AllocateScalars();
            videoMsg->SetScalarType(videoMsg->TYPE_UINT32);
            videoMsg->SetEndian(igtl_is_little_endian() == 1 ? 2 : 1); //little endian is 2 big endian is 1
            videoMsg->SetWidth(td->td_Server->pic.iPicWidth);
            videoMsg->SetHeight(td->td_Server->pic.iPicHeight);
            memcpy(videoMsg->GetPackFragmentPointer(2), td->td_Server->pic.pData[0]+ iMessage*pEncParamExt.iPicWidth*pEncParamExt.iPicHeight, pEncParamExt.iPicWidth*pEncParamExt.iPicHeight);
            videoMsg->Pack();
            glock->Lock();
            for (int i = 0; i < videoMsg->GetNumberOfPackFragments(); i++)
            {
              socket->Send(videoMsg->GetPackFragmentPointer(i), videoMsg->GetPackFragmentSize(i));
            }
            glock->Unlock();
          }
          rv = encoderColor_->EncodeFrame(&td->td_Server->pic_Color, &td->td_Server->info_Color);
          if (rv == cmResultSuccess)
          {
            //---------------
            igtl::VideoMessage::Pointer videoMsg;
            videoMsg = igtl::VideoMessage::New();
            videoMsg->SetDefaultBodyType("ColoredDepth");
            videoMsg->SetDeviceName("ColorFrame");
            videoMsg->SetBitStreamSize(td->td_Server->info_Color.iFrameSizeInBytes);
            videoMsg->AllocateScalars();
            videoMsg->SetScalarType(videoMsg->TYPE_UINT32);
            videoMsg->SetEndian(igtl_is_little_endian() == 1 ? 2 : 1); //little endian is 2 big endian is 1
            videoMsg->SetWidth(td->td_Server->pic_Color.iPicWidth);
            videoMsg->SetHeight(td->td_Server->pic_Color.iPicHeight);
            int frameSize = 0;
            int layerSize = 0;
            for (int i = 0; i < td->td_Server->info_Color.iLayerNum; ++i) {
              const SLayerBSInfo& layerInfo = td->td_Server->info_Color.sLayerInfo[i];
              layerSize = 0;
              for (int j = 0; j < layerInfo.iNalCount; ++j)
              {
                frameSize += layerInfo.pNalLengthInByte[j];
                layerSize += layerInfo.pNalLengthInByte[j];
              }
              //TestDebugCharArrayCmp(layerInfo.pBsBuf, layerInfo.pBsBuf, layerSize<200? layerSize:200);
              for (int i = 0; i < layerSize; i++)
              {
                videoMsg->GetPackFragmentPointer(2)[frameSize - layerSize + i] = layerInfo.pBsBuf[i];
              }
              //fwrite(layerInfo.pBsBuf, 1, layerSize, pFpBs); // write pure bit stream into file
            }
            //std::cerr<<"line break"<<std::endl;
            //TestDebugCharArrayCmp(videoMsg->GetPackFragmentPointer(2), videoMsg->GetPackFragmentPointer(1) + IGTL_VIDEO_HEADER_SIZE, 200);
            videoMsg->Pack();
            glock->Lock();

            for (int i = 0; i < videoMsg->GetNumberOfPackFragments(); i++)
            {
              socket->Send(videoMsg->GetPackFragmentPointer(i), videoMsg->GetPackFragmentSize(i));
            }
            glock->Unlock();
          }
          td->td_Server->transmissionFinished = true;
          td->td_Server->conditionVar->Signal();
          //igtl::Sleep(interval);
        }
        //------------------------------------------------------------
        // Loop
      }
    }
    WelsDestroySVCEncoder(encoder_);
    return NULL;
  }
}


