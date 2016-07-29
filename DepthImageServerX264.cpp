#include <iostream>

#include "igtl_header.h"
#include "igtl_video.h"
#include "utils/BufferedData.h"
#include "utils/FileInputStream.h"
#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlVideoMessage.h"
#include "igtlServerSocket.h"
#include "igtlMultiThreader.h"
#include "igtlConditionVariable.h"

extern "C" {
  #include "stdint.h"
  #include "x264.h"
  #include "x264cli.h"
  //#include "common\common.h"
}
#define FAIL_IF_ERROR( cond, ... ) FAIL_IF_ERR( cond, "x264", __VA_ARGS__ )

bool Synchonize = true;
bool useDemux = true;
int DemuxMethod = 2;
bool useCompressForRGB = false;

void* ThreadFunction(void* ptr);
int   SendVideoData(igtl::Socket::Pointer& socket, igtl::VideoMessage::Pointer& videoMsg);
namespace DepthImageServerX264 {
  
  typedef enum {
    videoFormatRGB = 1,             ///< rgb color formats
    videoFormatRGBA = 2,
    videoFormatRGB555 = 3,
    videoFormatRGB565 = 4,
    videoFormatBGR = 5,
    videoFormatBGRA = 6,
    videoFormatABGR = 7,
    videoFormatARGB = 8,

    videoFormatYUY2 = 20,            ///< yuv color formats
    videoFormatYVYU = 21,
    videoFormatUYVY = 22,
    videoFormatI420 = 23,            ///< the same as IYUV
    videoFormatYV12 = 24,
    videoFormatInternal = 25,            ///< only used in SVC decoder testbed

    videoFormatNV12 = 26,            ///< new format for output by DXVA decoding

    videoFormatVFlip = 0x80000000
  } EVideoFormatType;

  /**
  * @brief Enumerate  video frame type
  */
  typedef enum {
    videoFrameTypeInvalid,    ///< encoder not ready or parameters are invalidate
    videoFrameTypeIDR,        ///< IDR frame in H.264
    videoFrameTypeI,          ///< I frame type
    videoFrameTypeP,          ///< P frame type
    videoFrameTypeSkip,       ///< skip the frame based encoder kernel
    videoFrameTypeIPMixed     ///< a frame where I and P slices are mixing, not supported yet
  } EVideoFrameType;
  /**
  * @brief Bitstream inforamtion of a layer being encoded
  */
  typedef struct {
    unsigned char uiTemporalId;
    unsigned char uiSpatialId;
    unsigned char uiQualityId;
    EVideoFrameType eFrameType;
    unsigned char uiLayerType;

    /**
    * The sub sequence layers are ordered hierarchically based on their dependency on each other so that any picture in a layer shall not be
    * predicted from any picture on any higher layer.
    */
    int   iSubSeqId;                ///< refer to D.2.11 Sub-sequence information SEI message semantics
    int   iNalCount;              ///< count number of NAL coded already
    int*  pNalLengthInByte;       ///< length of NAL size in byte from 0 to iNalCount-1
    unsigned char*  pBsBuf;       ///< buffer of bitstream contained
  } SLayerBSInfo, *PLayerBSInfo;

  /**
  * @brief Frame bit stream info
  */
  typedef struct {
    int           iLayerNum;
    SLayerBSInfo  sLayerInfo[128];

    EVideoFrameType eFrameType;
    int iFrameSizeInBytes;
    long long uiTimeStamp;
  } SFrameBSInfo, *PFrameBSInfo;

  /**
  *  @brief Structure for source picture
  */
  typedef struct Source_Picture_s {
    int       iColorFormat;          ///< color space type
    int       iStride[4];            ///< stride for each plane pData
    unsigned char*  pData[4];        ///< plane pData
    int       iPicWidth;             ///< luma picture width in x coordinate
    int       iPicHeight;            ///< luma picture height in y coordinate
    long long uiTimeStamp;           ///< timestamp of the source picture, unit: millisecond
  } SSourcePicture;
  typedef struct {
    igtl::MutexLock::Pointer glock;
    int   stop;
    int   portNum;
    x264_picture_t pic_DepthFrame;
    x264_picture_t pic_DepthIndex;
    x264_picture_t pic_Color;
    bool transmissionFinished;
    igtl::ConditionVariable::Pointer conditionVar;
  } ThreadDataServer;
  typedef struct {
    int   nloop;
    igtl::MutexLock::Pointer glock;
    igtl::Socket::Pointer socket;
    int   interval;
    int   stop;
    bool  useCompression;
    ThreadDataServer* td_Server;
  } ThreadData;
}
typedef struct {
  int b_progress;
  int i_seek;
  FILE *qpfile;
  FILE *tcfile_out;
  double timebase_convert_multiplier;
  int i_pulldown;
} cli_opt_t;

typedef struct
{
  int mod;
  uint8_t pattern[24];
  float fps_factor;
} cli_pulldown_t;
#define TB  PIC_STRUCT_TOP_BOTTOM
#define BT  PIC_STRUCT_BOTTOM_TOP
#define TBT PIC_STRUCT_TOP_BOTTOM_TOP
#define BTB PIC_STRUCT_BOTTOM_TOP_BOTTOM
// indexed by pic_struct enum
static const float pulldown_frame_duration[10] = { 0.0, 1, 0.5, 0.5, 1, 1, 1.5, 1.5, 2, 3 };

static const char * const demuxer_names[] =
{
  "auto",
  "raw",
  "y4m",
#if HAVE_AVS
  "avs",
#endif
#if HAVE_LAVF
  "lavf",
#endif
#if HAVE_FFMS
  "ffms",
#endif
  0
};

static const char * const muxer_names[] =
{
  "auto",
  "raw",
  "mkv",
  "flv",
#if HAVE_GPAC || HAVE_LSMASH
  "mp4",
#endif
  0
};

static const char * const pulldown_names[] = { "none", "22", "32", "64", "double", "triple", "euro", 0 };
static const char * const log_level_names[] = { "none", "error", "warning", "info", "debug", 0 };
static const char * const output_csp_names[] =
{
#if !X264_CHROMA_FORMAT || X264_CHROMA_FORMAT == X264_CSP_I420
  "i420",
#endif
#if !X264_CHROMA_FORMAT || X264_CHROMA_FORMAT == X264_CSP_I422
  "i422",
#endif
#if !X264_CHROMA_FORMAT || X264_CHROMA_FORMAT == X264_CSP_I444
  "i444", "rgb",
#endif
  0
};

static const char * const range_names[] = { "auto", "tv", "pc", 0 };

/* logging and printing for within the cli system */
static int cli_log_level;
void x264_cli_log( const char *name, int i_level, const char *fmt, ... )
{
  if( i_level > cli_log_level )
    return;
  char *s_level;
  switch( i_level )
  {
    case X264_LOG_ERROR:
      s_level = "error";
      break;
    case X264_LOG_WARNING:
      s_level = "warning";
      break;
    case X264_LOG_INFO:
      s_level = "info";
      break;
    case X264_LOG_DEBUG:
      s_level = "debug";
      break;
    default:
      s_level = "unknown";
      break;
  }
  fprintf( stderr, "%s [%s]: ", name, s_level );
  va_list arg;
  va_start( arg, fmt );
  x264_vfprintf( stderr, fmt, arg );
  va_end( arg );
}

typedef enum
{
  OPT_FRAMES = 256,
  OPT_SEEK,
  OPT_QPFILE,
  OPT_THREAD_INPUT,
  OPT_QUIET,
  OPT_NOPROGRESS,
  OPT_LONGHELP,
  OPT_PROFILE,
  OPT_PRESET,
  OPT_TUNE,
  OPT_SLOWFIRSTPASS,
  OPT_FULLHELP,
  OPT_FPS,
  OPT_MUXER,
  OPT_DEMUXER,
  OPT_INDEX,
  OPT_INTERLACED,
  OPT_TCFILE_IN,
  OPT_TIMEBASE,
  OPT_PULLDOWN,
  OPT_LOG_LEVEL,
  OPT_VIDEO_FILTER,
  OPT_INPUT_FMT,
  OPT_INPUT_RES,
  OPT_INPUT_CSP,
  OPT_INPUT_DEPTH,
  OPT_DTS_COMPRESSION,
  OPT_OUTPUT_CSP,
  OPT_INPUT_RANGE,
  OPT_RANGE
} OptionsOPT;

static char const *strtable_lookup( const char * const table[], int idx )
{
  int i = 0; while( table[i] ) i++;
  return ( ( idx >= 0 && idx < i ) ? table[ idx ] : "???" );
}

static char *stringify_names( char *buf, const char * const names[] )
{
  int i = 0;
  char *p = buf;
  for( p[0] = 0; names[i]; i++ )
  {
    p += sprintf( p, "%s", names[i] );
    if( names[i+1] )
      p += sprintf( p, ", " );
  }
  return buf;
}

static char short_options[] = "8A:B:b:f:hI:i:m:o:p:q:r:t:Vvw";

static int parse_enum_name( const char *arg, const char * const *names, const char **dst )
{
  for( int i = 0; names[i]; i++ )
    if( !strcmp( arg, names[i] ) )
    {
      *dst = names[i];
      return 0;
    }
  return -1;
}

static int parse_enum_value( const char *arg, const char * const *names, int *dst )
{
  for( int i = 0; names[i]; i++ )
    if( !strcmp( arg, names[i] ) )
    {
      *dst = i;
      return 0;
    }
  return -1;
}
static int parse( int argc, char **argv, x264_param_t *param, cli_opt_t *opt )
{
  const char *demuxer = demuxer_names[0];
  char *output_filename = NULL;
  const char *muxer = muxer_names[0];
  x264_param_t defaults;
  char *profile = NULL;
  char *vid_filters = NULL;
  int b_thread_input = 0;
  int b_turbo = 1;
  int b_user_ref = 0;
  int b_user_fps = 0;
  int b_user_interlaced = 0;
  char *preset = NULL;
  char *tune = NULL;
  
  x264_param_default( &defaults );
  int output_csp = defaults.i_csp;
  opt->b_progress = 1;
  
  if( preset && !strcmp( preset, "placebo" ) )
    b_turbo = 0;
  
  if( x264_param_default_preset( param, preset, tune ) < 0 )
    return -1;
  
  /* If first pass mode is used, apply faster settings. */
  if( b_turbo )
    x264_param_apply_fastfirstpass( param );
  
  /* Apply profile restrictions. */
  if( x264_param_apply_profile( param, profile ) < 0 )
    return -1;
  /* if the user never specified the output range and the input is now rgb, default it to pc */
  int csp = param->i_csp & X264_CSP_MASK;
  if( csp >= X264_CSP_BGR && csp <= X264_CSP_RGB )
  {
      param->vui.b_fullrange = RANGE_PC;
  }
 
  return 0;
}
std::string     videoFile = "";

void ServerControl(void * ptr)
{
  //------------------------------------------------------------
  // Parse Arguments
 
  int    port     = 18944;
  
  igtl::MutexLock::Pointer glock = igtl::MutexLock::New();
  igtl::MultiThreader::Pointer threader = igtl::MultiThreader::New();
  igtl::MultiThreader::ThreadInfo* info =
  static_cast<igtl::MultiThreader::ThreadInfo*>(ptr);

  //int id      = info->ThreadID;
  //int nThread = info->NumberOfThreads;
  DepthImageServerX264::ThreadData* td = static_cast<DepthImageServerX264::ThreadData*>(info->UserData);

  threader->SpawnThread((igtl::ThreadFunctionType) &ThreadFunction, td);
  while(1);
  {
    igtl::Sleep(10000);
  }
  
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);
  
  if (r < 0)
  {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
  }
  while(1){
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
            td->stop = 1;
            threader->TerminateThread(threadID);
            threadID = -1;
          }
          std::cerr << "Disconnecting the client." << std::endl;
          td->socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
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
            td->interval = startVideoMsg->GetResolution();
            td->glock    = glock;
            td->socket   = socket;
            td->stop     = 0;
            td->useCompression = startVideoMsg->GetUseCompress();
            threadID    = threader->SpawnThread((igtl::ThreadFunctionType) &ThreadFunction, &td);
          }
        }
        else if (strcmp(headerMsg->GetDeviceType(), "STP_VIDEO") == 0)
        {
          socket->Skip(headerMsg->GetBodySizeToRead(), 0);
          std::cerr << "Received a STP_VIDEO message." << std::endl;
          if (threadID >= 0)
          {
            td->stop  = 1;
            threader->TerminateThread(threadID);
            threadID = -1;
            std::cerr << "Disconnecting the client." << std::endl;
            td->socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
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
      //memcpy(pYUVBuf, yuvImg.data, bufLen*sizeof(unsigned char));
      //fwrite(pYUVBuf, bufLen*sizeof(unsigned char), 1, pFileOut);
  }
}

void* ThreadFunction(void* ptr)
{
  //------------------------------------------------------------
  // Get thread information
  igtl::MultiThreader::ThreadInfo* info =
  static_cast<igtl::MultiThreader::ThreadInfo*>(ptr);
  
  //int id      = info->ThreadID;
  //int nThread = info->NumberOfThreads;
  DepthImageServerX264::ThreadData* td = static_cast<DepthImageServerX264::ThreadData*>(info->UserData);
  
  //------------------------------------------------------------
  // Get user data
  igtl::MutexLock::Pointer glock = td->glock;
  long interval = td->interval;
  std::cerr << "Interval = " << interval << " (ms)" << std::endl;
  //long interval = 1000;
  //long interval = (id + 1) * 100; // (ms)
  
  igtl::Socket::Pointer& socket = td->socket;
  
  //------------------------------------------------------------
  // Allocate TrackingData Message Class
  //
  // NOTE: TrackingDataElement class instances are allocated
  //       before the loop starts to avoid reallocation
  //       in each image transfer.
  int picWidth = 512, picHeight = 424;
  x264_param_t param;
  x264_picture_t pic_out;
  x264_nal_t *nal;
  int i_nal;
  x264_param_default_preset( &param, "medium", NULL );
  cli_opt_t opt;
  x264_t *h_DepthFrame = NULL;
  x264_t *h_DepthIndex= NULL;
  x264_t *h_ColorFrame = NULL;
  
  param.i_width  = picWidth;
  param.i_height = picHeight;
  param.b_vfr_input = 0;
  param.b_repeat_headers = 1;
  param.b_annexb = 1;
  x264_param_apply_profile( &param, "high444" );
  int argc = 7;
  char *argv[7] = {"","--qp","0","--crf","24", "--tune", "zerolatency"};
  const cli_pulldown_t *pulldown = NULL; // shut up gcc
  
  int     i_frame = 0;
#   define  MAX_PTS_WARNING 3 /* arbitrary */
  int     pts_warning_cnt = 0;
  int64_t largest_pts = -1;
  int64_t second_largest_pts = -1;
  int64_t ticks_per_frame;
  double  pulldown_pts = 0;
  
  opt.b_progress &= param.i_log_level < X264_LOG_DEBUG;

  parse( argc, argv, &param, &opt ) ;
  param.rc.i_rc_method = 0;
  param.i_csp = X264_CSP_BGR;
  h_DepthFrame = x264_encoder_open(&param);
  h_DepthIndex = x264_encoder_open(&param);
  h_ColorFrame = x264_encoder_open(&param);
  x264_t* h[3] = {h_DepthFrame,h_DepthIndex, h_ColorFrame};
  x264_picture_t* pictureGroup[3] = {&td->td_Server->pic_DepthFrame,&td->td_Server->pic_DepthIndex, &td->td_Server->pic_Color};
  //x264_encoder_parameters(h_DepthFrame, &param);
  //x264_encoder_parameters(h_DepthIndex, &param);
  //x264_encoder_parameters(h_ColorFrame, &param);
  ticks_per_frame = (int64_t)param.i_timebase_den * param.i_fps_den / param.i_timebase_num / param.i_fps_num;
  ticks_per_frame = X264_MAX( ticks_per_frame, 1 );
  while (!td->stop)
  {
    if (!param.b_vfr_input)
    {
      for(int i = 0; i<3 ;i++)
        pictureGroup[i]->i_pts = i_frame;
    }
    if( opt.i_pulldown && !param.b_vfr_input )
    {
      for (int i = 0; i < 3; i++)
      {
        pictureGroup[i]->i_pic_struct = pulldown->pattern[i_frame % pulldown->mod];
        pictureGroup[i]->i_pts = (int64_t)(pulldown_pts + 0.5);
        pulldown_pts += pulldown_frame_duration[pictureGroup[i]->i_pic_struct];
      }
      
    }
    else if (opt.timebase_convert_multiplier)
    {
      for (int i = 0; i < 3; i++)
      {
        pictureGroup[i]->i_pts = (int64_t)(pictureGroup[i]->i_pts * opt.timebase_convert_multiplier + 0.5);
      }
    }
    std::string frameNames[3] = { "DepthFrame", "DepthIndex","ColorFrame"};
    for (int iMessage = 0; iMessage < 3; iMessage++)
    {
      int i_frame_size = x264_encoder_encode(h[iMessage], &nal, &i_nal, pictureGroup[iMessage], &pic_out);
      if (i_frame_size > 0)
      {
        igtl::VideoMessage::Pointer videoMsg;
        videoMsg = igtl::VideoMessage::New();
        videoMsg->SetDefaultBodyType("ColoredDepth");
        videoMsg->SetDeviceName(frameNames[iMessage]);
        videoMsg->SetBitStreamSize(picWidth*picHeight);
        videoMsg->AllocateScalars();
        videoMsg->SetScalarType(videoMsg->TYPE_UINT32);
        videoMsg->SetEndian(igtl_is_little_endian() == 1 ? 2 : 1); //little endian is 2 big endian is 1
        videoMsg->SetWidth(picWidth);
        videoMsg->SetHeight(picHeight);
        memcpy(videoMsg->GetPackFragmentPointer(2), nal[0].p_payload, i_frame_size);
        videoMsg->Pack();
        glock->Lock();
        for (int i = 0; i < videoMsg->GetNumberOfPackFragments(); i++)
        {
          socket->Send(videoMsg->GetPackFragmentPointer(i), videoMsg->GetPackFragmentSize(i));
        }
        glock->Unlock();
      }
    }
    i_frame++;
    td->td_Server->transmissionFinished = true;
    td->td_Server->conditionVar->Signal();
  }
  x264_encoder_close(h_DepthIndex);
  x264_encoder_close(h_DepthFrame);
  x264_encoder_close(h_ColorFrame);
  return NULL;
}

