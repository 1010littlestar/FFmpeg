#include "libavutil/common.h"
#include "libavutil/fifo.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libswscale/swscale.h"

#include "avcodec.h"
#include "internal.h"

#include <sys/types.h>
#include <sys/poll.h>
#include <sys/time.h>

#include <hi_common.h>
#include <hi_buffer.h>
#include <hi_comm_sys.h>
#include <hi_comm_vb.h>
#include <hi_comm_vdec.h>
#include <hi_comm_vpss.h>
#include <hi_comm_vo.h>
#include <hi_defines.h>
#include <hi_comm_vgs.h>
#include <hi_comm_hdmi.h>

#include <mpi_sys.h>
#include <mpi_vb.h>
#include <mpi_vdec.h>
#include <mpi_region.h>
#include <mpi_vgs.h>
#include <mpi_vpss.h>
#include <mpi_vo.h>
#include <mpi_hdmi.h>

#include <sample_comm.h>

typedef struct HiMppContext {
    // 解码模块参数
    HI_U32 u32VdecChnNum;

    HI_U32 VpssGrpNum;

    // 输出模块参数
    SAMPLE_VO_CONFIG_S stVoConfig;

}HiMppContext;

static av_cold int hi_decode_init(AVCodecContext *avctx)
{
    HiMppContext *ctx = avctx->priv_data;

    int i;
    VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_CHN_NUM];
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];
    SAMPLE_VO_CONFIG_S *pstVoConfig = &ctx->stVoConfig;

    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32VdecChnNum = 8;
    HI_U32 VpssGrpNum = 8;
    PIC_SIZE_E enDispPicSize;
    SIZE_S stDispSize;
    VB_CONFIG_S stVbConfig;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    

    avctx->width = 1920;
    avctx->height = 1080;
    ctx->u32VdecChnNum = u32VdecChnNum;
    ctx->VpssGrpNum = VpssGrpNum;

    av_log(avctx, AV_LOG_DEBUG, "video width:%d, height:%d", avctx->width, avctx->height);
    // 帧尺寸设置
    enDispPicSize = PIC_1080P;

    s32Ret =  SAMPLE_COMM_SYS_GetPicSize(enDispPicSize, &stDispSize);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "sys get pic size fail for %#x!", s32Ret);
        goto END1;
    }

    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/
    memset(&stVbConfig, 0, sizeof(VB_CONFIG_S));
    stVbConfig.u32MaxPoolCnt             = 1;
    stVbConfig.astCommPool[0].u32BlkCnt  = 10 * u32VdecChnNum;
    stVbConfig.astCommPool[0].u64BlkSize = COMMON_GetPicBufferSize(stDispSize.u32Width, stDispSize.u32Height,
                                                PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8, COMPRESS_MODE_SEG, 0);
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConfig);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "init sys fail for %#x!", s32Ret);
        goto END1;
    }

    /************************************************
    step2:  init module VB or user VB(for VDEC)
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        astSampleVdec[i].enType                           = PT_H264;
        astSampleVdec[i].u32Width                         = 1920;
        astSampleVdec[i].u32Height                        = 1080;
        astSampleVdec[i].enMode                           = VIDEO_MODE_FRAME;
        astSampleVdec[i].stSapmleVdecVideo.enDecMode      = VIDEO_DEC_MODE_IPB;
        astSampleVdec[i].stSapmleVdecVideo.enBitWidth     = DATA_BITWIDTH_8;
        astSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum = 3;
        astSampleVdec[i].u32DisplayFrameNum               = 2;
        astSampleVdec[i].u32FrameBufCnt = astSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum + astSampleVdec[i].u32DisplayFrameNum + 1;
    }
    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "start VDEC fail for %#x!\n", s32Ret);
        goto END3;
    }

    /************************************************
    step4:  start VPSS
    *************************************************/
    stVpssGrpAttr.u32MaxW = 1920;
    stVpssGrpAttr.u32MaxH = 1080;
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate = -1;
    stVpssGrpAttr.enDynamicRange = DYNAMIC_RANGE_SDR8;
    stVpssGrpAttr.enPixelFormat  = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssGrpAttr.bNrEn   = HI_FALSE;

    memset(abChnEnable, 0, sizeof(abChnEnable));
    abChnEnable[0] = HI_TRUE;
    astVpssChnAttr[0].u32Width                    = stDispSize.u32Width;
    astVpssChnAttr[0].u32Height                   = stDispSize.u32Height;
    astVpssChnAttr[0].enChnMode                   = VPSS_CHN_MODE_AUTO;
    astVpssChnAttr[0].enCompressMode              = COMPRESS_MODE_SEG;
    astVpssChnAttr[0].enDynamicRange              = DYNAMIC_RANGE_SDR8;
    astVpssChnAttr[0].enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    astVpssChnAttr[0].stFrameRate.s32SrcFrameRate = -1;
    astVpssChnAttr[0].stFrameRate.s32DstFrameRate = -1;
    astVpssChnAttr[0].u32Depth                    = 0;
    astVpssChnAttr[0].bMirror                     = HI_FALSE;
    astVpssChnAttr[0].bFlip                       = HI_FALSE;
    astVpssChnAttr[0].stAspectRatio.enMode        = ASPECT_RATIO_NONE;
    astVpssChnAttr[0].enVideoFormat               = VIDEO_FORMAT_LINEAR;

    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_Start(i, &abChnEnable[0], &stVpssGrpAttr, &astVpssChnAttr[0]);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "start VPSS %d fail for %#x!\n", i, s32Ret);
            goto END4;
        }
    }

    /************************************************
    step5:  start VO
    *************************************************/
    pstVoConfig->VoDev                 = SAMPLE_VO_DEV_UHD;
    pstVoConfig->enVoIntfType          = VO_INTF_HDMI;
    pstVoConfig->enIntfSync            = VO_OUTPUT_1080P60;
    pstVoConfig->enPicSize             = enDispPicSize;
    pstVoConfig->u32BgColor            = COLOR_RGB_BLUE;
    pstVoConfig->u32DisBufLen          = 3;
    pstVoConfig->enDstDynamicRange     = DYNAMIC_RANGE_SDR8;
    pstVoConfig->enVoMode              = VO_MODE_8MUX;
    pstVoConfig->enPixFormat           = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    pstVoConfig->stDispRect.s32X       = 0;
    pstVoConfig->stDispRect.s32Y       = 0;
    pstVoConfig->stDispRect.u32Width   = stDispSize.u32Width;
    pstVoConfig->stDispRect.u32Height  = stDispSize.u32Height;
    pstVoConfig->stImageSize.u32Width  = stDispSize.u32Width;
    pstVoConfig->stImageSize.u32Height = stDispSize.u32Height;
    pstVoConfig->enVoPartMode          = VO_PART_MODE_SINGLE;
    s32Ret = SAMPLE_COMM_VO_StartVO(pstVoConfig);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "start VO fail for %#x!\n", s32Ret);
        goto END5;
    }

    /************************************************
    step6:  VDEC bind VPSS
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_Bind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "vdec bind vpss fail for %#x!\n", s32Ret);
            goto END6;
        }
    }

    /************************************************
    step7:  VPSS bind VO
    *************************************************/
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_Bind_VO(i, 0, pstVoConfig->VoDev, i);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "vpss bind vo fail for %#x!\n", s32Ret);
            goto END7;
        }
    }

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    return 0;

END7:
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_UnBind_VO(i, 0, pstVoConfig->VoDev, i);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "vpss unbind vo fail for %#x!\n", s32Ret);
        }
    }

END6:
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_UnBind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "vdec unbind vpss fail for %#x!\n", s32Ret);
        }
    }

END5:
    SAMPLE_COMM_VO_StopVO(pstVoConfig);

END4:
    for(i=0; i<VpssGrpNum; i++)
    {
        memset(abChnEnable, 0, sizeof(abChnEnable));
        abChnEnable[0] = HI_TRUE;

        SAMPLE_COMM_VPSS_Stop(i, &abChnEnable[0]);
    }

END3:
    SAMPLE_COMM_VDEC_Stop(u32VdecChnNum);

END2:
    SAMPLE_COMM_VDEC_ExitVBPool();

END1:
    SAMPLE_COMM_SYS_Exit();

    return -1;
}

static int hi_decode_frame(AVCodecContext *avctx, void *data, 
                          int *got_frame, AVPacket *avpkt)
{
    HI_S32 s32Ret = HI_SUCCESS;

    VDEC_STREAM_S stStream;

    av_log(avctx, AV_LOG_TRACE, "hi_decode_frame: width:%d, height:%d, pts:%ld, size:%u\n", 
           avctx->width, avctx->height, avpkt->pts, avpkt->size);
    stStream.u64PTS       = avpkt->pts;
    stStream.pu8Addr      = avpkt->data;
    stStream.u32Len       = avpkt->size;
    stStream.bEndOfFrame  = HI_TRUE;
    stStream.bEndOfStream = !avpkt->data ? HI_TRUE : HI_FALSE;
    stStream.bDisplay     = 1;

    s32Ret = HI_MPI_VDEC_SendStream(0, &stStream, 0);
    if((HI_SUCCESS != s32Ret))
    {
      av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VDEC_SendStream fail for %#x!\n", s32Ret);
    }

    return s32Ret;
}

static av_cold int hi_decode_close(AVCodecContext *avctx)
{
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];
    HiMppContext *ctx = avctx->priv_data;
    HI_U32 VpssGrpNum = ctx->VpssGrpNum;
    HI_U32 u32VdecChnNum = ctx->u32VdecChnNum;
    SAMPLE_VO_CONFIG_S *pstVoConfig = &ctx->stVoConfig;
    HI_S32 s32Ret = HI_SUCCESS;
    int i;

    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_UnBind_VO(i, 0, pstVoConfig->VoDev, i);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "vpss unbind vo fail for %#x!\n", s32Ret);
        }
    }

    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_UnBind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            av_log(avctx, AV_LOG_DEBUG, "vdec unbind vpss fail for %#x!\n", s32Ret);
        }
    }

    SAMPLE_COMM_VO_StopVO(pstVoConfig);

    for(i=0; i<VpssGrpNum; i++)
    {
        memset(abChnEnable, 0, sizeof(abChnEnable));
        abChnEnable[0] = HI_TRUE;

        SAMPLE_COMM_VPSS_Stop(i, &abChnEnable[0]);
    }

    SAMPLE_COMM_VDEC_Stop(u32VdecChnNum);

    SAMPLE_COMM_VDEC_ExitVBPool();

    SAMPLE_COMM_SYS_Exit();
  
    av_log(avctx, AV_LOG_DEBUG, "himpp_decode_close");

  return 0;
}


AVCodec ff_h264_himpp_decoder = {
    .name           = "h264_himpp",
    .long_name      = NULL_IF_CONFIG_SMALL("hisi mpp H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H264,
    .priv_data_size = sizeof(HiMppContext),
    .init           = hi_decode_init,
    .decode         = hi_decode_frame,
    .close          = hi_decode_close,
    .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_SETS_PKT_DTS | FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
    .wrapper_name   = "himpp",
};
