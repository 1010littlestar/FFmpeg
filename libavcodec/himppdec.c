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
#include <hi_defines.h>
#include <hi_comm_vgs.h>

#include <mpi_sys.h>
#include <mpi_vb.h>
#include <mpi_vdec.h>
#include <mpi_region.h>
#include <mpi_vgs.h>


#define SAMPLE_PRT(fmt...)   \
    do {\
        printf("[%s]-%d: ", __FUNCTION__, __LINE__);\
        printf(fmt);\
    }while(0)

#define CHECK_NULL_PTR(ptr)\
    do{\
        if(NULL == ptr)\
        {\
            printf("func:%s,line:%d, NULL pointer\n",__FUNCTION__,__LINE__);\
            return HI_FAILURE;\
        }\
    }while(0)

#define CHECK_CHN_RET(express,Chn,name)\
    do{\
        HI_S32 Ret;\
        Ret = express;\
        if (HI_SUCCESS != Ret)\
        {\
            printf("\033[0;31m%s chn %d failed at %s: LINE: %d with %#x!\033[0;39m\n", name, Chn, __FUNCTION__, __LINE__, Ret);\
            fflush(stdout);\
            return Ret;\
        }\
    }while(0)

#define CHECK_RET(express,name)\
    do{\
        HI_S32 Ret;\
        Ret = express;\
        if (HI_SUCCESS != Ret)\
        {\
            printf("\033[0;31m%s failed at %s: LINE: %d with %#x!\033[0;39m\n", name, __FUNCTION__, __LINE__, Ret);\
            return Ret;\
        }\
    }while(0)


typedef struct hiSAMPLE_VDEC_BUF
{
    HI_U32  u32PicBufSize;
    HI_U32  u32TmvBufSize;
    HI_BOOL bPicBufAlloc;
    HI_BOOL bTmvBufAlloc;
}SAMPLE_VDEC_BUF;

typedef struct hiSAMPLE_VDEC_VIDEO_ATTR
{
    VIDEO_DEC_MODE_E enDecMode;
    HI_U32              u32RefFrameNum;
    DATA_BITWIDTH_E  enBitWidth;
}SAMPLE_VDEC_VIDEO_ATTR;

typedef struct hiSAMPLE_VDEC_PICTURE_ATTR
{
    PIXEL_FORMAT_E enPixelFormat;
    HI_U32         u32Alpha;
}SAMPLE_VDEC_PICTURE_ATTR;

typedef struct hiSAMPLE_VDEC_ATTR
{
    PAYLOAD_TYPE_E enType;
    VIDEO_MODE_E   enMode;
    HI_U32 u32Width;
    HI_U32 u32Height;
    HI_U32 u32FrameBufCnt;
    HI_U32 u32DisplayFrameNum;
    union
    {
        SAMPLE_VDEC_VIDEO_ATTR stSapmleVdecVideo;      /* structure with video ( h265/h264) */
        SAMPLE_VDEC_PICTURE_ATTR stSapmleVdecPicture; /* structure with picture (jpeg/mjpeg )*/
    };
}SAMPLE_VDEC_ATTR;


typedef struct HiContext {
    HI_U32 u32VdecChnNum;
    VB_CONFIG_S stVbConfig;
    SAMPLE_VDEC_ATTR stHiMppVdec;
}HiContext;


VB_SOURCE_E  g_enVdecVBSource  = VB_SOURCE_MODULE;

VB_POOL g_ahPicVbPool[VB_MAX_POOLS] = {[0 ... (VB_MAX_POOLS-1)] = VB_INVALID_POOLID};
VB_POOL g_ahTmvVbPool[VB_MAX_POOLS] = {[0 ... (VB_MAX_POOLS-1)] = VB_INVALID_POOLID};


static HI_S32 SAMPLE_COMM_SYS_Init(VB_CONFIG_S* pstVbConfig)
{
    HI_S32 s32Ret = HI_FAILURE;

    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();

    if (NULL == pstVbConfig)
    {
        SAMPLE_PRT("input parameter is null, it is invaild!\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_VB_SetConfig(pstVbConfig);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VB_SetConf failed!\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_VB_Init();

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VB_Init failed!\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_SYS_Init();

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_SYS_Init failed!\n");
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

static HI_S32 SAMPLE_COMM_VDEC_InitVBPool(HI_U32 ChnNum, SAMPLE_VDEC_ATTR *pastSampleVdec)
{
    VB_CONFIG_S stVbConf;
    HI_S32 i, j, pos=0, s32Ret;
    HI_BOOL bFindFlag;
    SAMPLE_VDEC_BUF astSampleVdecBuf[VDEC_MAX_CHN_NUM];
    VB_POOL_CONFIG_S stVbPoolCfg;

    memset(astSampleVdecBuf, 0, sizeof(SAMPLE_VDEC_BUF)*VDEC_MAX_CHN_NUM);
    memset(&stVbConf, 0, sizeof(VB_CONFIG_S));

    for(i=0; i<ChnNum; i++)
    {
        if(PT_H265 == pastSampleVdec[i].enType)
        {
            astSampleVdecBuf[i].u32PicBufSize = VDEC_GetPicBufferSize(pastSampleVdec[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height,
                                                    PIXEL_FORMAT_YVU_SEMIPLANAR_420, pastSampleVdec[i].stSapmleVdecVideo.enBitWidth, 0);
            astSampleVdecBuf[i].u32TmvBufSize = VDEC_GetTmvBufferSize(pastSampleVdec[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height);
        }
        else if(PT_H264 == pastSampleVdec[i].enType)
        {
            astSampleVdecBuf[i].u32PicBufSize = VDEC_GetPicBufferSize(pastSampleVdec[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height,
                                                    PIXEL_FORMAT_YVU_SEMIPLANAR_420, pastSampleVdec[i].stSapmleVdecVideo.enBitWidth, 0);
            if(VIDEO_DEC_MODE_IPB == pastSampleVdec[i].stSapmleVdecVideo.enDecMode)
            {
                astSampleVdecBuf[i].u32TmvBufSize = VDEC_GetTmvBufferSize(pastSampleVdec[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height);
            }
        }
        else
        {
            astSampleVdecBuf[i].u32PicBufSize = VDEC_GetPicBufferSize(pastSampleVdec[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height,
                                                    pastSampleVdec[i].stSapmleVdecPicture.enPixelFormat, DATA_BITWIDTH_8, 0);
        }
    }

    /* PicBuffer */
    for(j=0; j<VB_MAX_COMM_POOLS; j++)
    {
        bFindFlag = HI_FALSE;
        for(i=0; i<ChnNum; i++)
        {
            if((HI_FALSE == bFindFlag) && (0 != astSampleVdecBuf[i].u32PicBufSize) && (HI_FALSE == astSampleVdecBuf[i].bPicBufAlloc) )
            {
                stVbConf.astCommPool[j].u64BlkSize = astSampleVdecBuf[i].u32PicBufSize;
                stVbConf.astCommPool[j].u32BlkCnt  = pastSampleVdec[i].u32FrameBufCnt;
                astSampleVdecBuf[i].bPicBufAlloc   = HI_TRUE;
                bFindFlag                          = HI_TRUE;
                pos = j;
            }

            if((HI_TRUE == bFindFlag) && (HI_FALSE == astSampleVdecBuf[i].bPicBufAlloc)
                && (stVbConf.astCommPool[j].u64BlkSize == astSampleVdecBuf[i].u32PicBufSize) )
            {
                stVbConf.astCommPool[j].u32BlkCnt += pastSampleVdec[i].u32FrameBufCnt;
                astSampleVdecBuf[i].bPicBufAlloc   = HI_TRUE;
            }
        }
    }

    /* TmvBuffer */
    for(j=pos+1; j<VB_MAX_COMM_POOLS; j++)
    {
        bFindFlag = HI_FALSE;
        for(i=0; i<ChnNum; i++)
        {
            if((HI_FALSE == bFindFlag) && (0 != astSampleVdecBuf[i].u32TmvBufSize) && (HI_FALSE == astSampleVdecBuf[i].bTmvBufAlloc) )
            {
                stVbConf.astCommPool[j].u64BlkSize = astSampleVdecBuf[i].u32TmvBufSize;
                stVbConf.astCommPool[j].u32BlkCnt  = pastSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum+1;
                astSampleVdecBuf[i].bTmvBufAlloc   = HI_TRUE;
                bFindFlag                          = HI_TRUE;
                pos = j;
            }

            if((HI_TRUE == bFindFlag) && (HI_FALSE == astSampleVdecBuf[i].bTmvBufAlloc)
                && (stVbConf.astCommPool[j].u64BlkSize == astSampleVdecBuf[i].u32TmvBufSize) )
            {
                stVbConf.astCommPool[j].u32BlkCnt += pastSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum+1;
                astSampleVdecBuf[i].bTmvBufAlloc   = HI_TRUE;
            }
        }
    }
    stVbConf.u32MaxPoolCnt = pos + 1;

    if(VB_SOURCE_MODULE == g_enVdecVBSource)
    {
        HI_MPI_VB_ExitModCommPool(VB_UID_VDEC);
        CHECK_RET(HI_MPI_VB_SetModPoolConfig(VB_UID_VDEC, &stVbConf), "HI_MPI_VB_SetModPoolConfigig");
        s32Ret = HI_MPI_VB_InitModCommPool(VB_UID_VDEC);
        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_VB_InitModCommPool fail for 0x%x\n", s32Ret);
            HI_MPI_VB_ExitModCommPool(VB_UID_VDEC);
            return HI_FAILURE;
        }
    }
    else if (VB_SOURCE_USER == g_enVdecVBSource)
    {
        for (i = 0; i < ChnNum; i++)
        {
            if ( (0 != astSampleVdecBuf[i].u32PicBufSize) && (0 != pastSampleVdec[i].u32FrameBufCnt))
            {
                memset(&stVbPoolCfg, 0, sizeof(VB_POOL_CONFIG_S));
                stVbPoolCfg.u64BlkSize  = astSampleVdecBuf[i].u32PicBufSize;
                stVbPoolCfg.u32BlkCnt   = pastSampleVdec[i].u32FrameBufCnt;
                stVbPoolCfg.enRemapMode = VB_REMAP_MODE_NONE;
                g_ahPicVbPool[i] = HI_MPI_VB_CreatePool(&stVbPoolCfg);
                if (VB_INVALID_POOLID == g_ahPicVbPool[i])
                {
                    goto fail;
                }
            }
            if (0 != astSampleVdecBuf[i].u32TmvBufSize)
            {
                memset(&stVbPoolCfg, 0, sizeof(VB_POOL_CONFIG_S));
                stVbPoolCfg.u64BlkSize  = astSampleVdecBuf[i].u32TmvBufSize;
                stVbPoolCfg.u32BlkCnt   = pastSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum+1;
                stVbPoolCfg.enRemapMode = VB_REMAP_MODE_NONE;
                g_ahTmvVbPool[i] = HI_MPI_VB_CreatePool(&stVbPoolCfg);
                if (VB_INVALID_POOLID == g_ahTmvVbPool[i])
                {
                    goto fail;
                }
            }
        }
    }

    return HI_SUCCESS;

    fail:
        for (;i>=0;i--)
        {
            if (VB_INVALID_POOLID != g_ahPicVbPool[i])
            {
                s32Ret = HI_MPI_VB_DestroyPool(g_ahPicVbPool[i]);
                if(HI_SUCCESS != s32Ret)
                {
                    printf("HI_MPI_VB_DestroyPool %d fail!\n",g_ahPicVbPool[i]);
                }
                g_ahPicVbPool[i] = VB_INVALID_POOLID;
            }
            if (VB_INVALID_POOLID != g_ahTmvVbPool[i])
            {
                s32Ret = HI_MPI_VB_DestroyPool(g_ahTmvVbPool[i]);
                if(HI_SUCCESS != s32Ret)
                {
                    printf("HI_MPI_VB_DestroyPool %d fail!\n",g_ahTmvVbPool[i]);
                }
                g_ahTmvVbPool[i] = VB_INVALID_POOLID;
            }
        }
        return HI_FAILURE;
}

static HI_S32 SAMPLE_COMM_VDEC_Start(HI_S32 s32ChnNum, SAMPLE_VDEC_ATTR *pastSampleVdec)
{
    HI_S32  i;
    VDEC_CHN_ATTR_S stChnAttr[VDEC_MAX_CHN_NUM];
    VDEC_CHN_POOL_S stPool;
    VDEC_CHN_PARAM_S stChnParam;
    VDEC_MOD_PARAM_S stModParam;

    CHECK_RET(HI_MPI_VDEC_GetModParam(&stModParam), "HI_MPI_VDEC_GetModParam");

    stModParam.enVdecVBSource = g_enVdecVBSource;
    CHECK_RET(HI_MPI_VDEC_SetModParam(&stModParam), "HI_MPI_VDEC_GetModParam");

    for(i=0; i<s32ChnNum; i++)
    {
        stChnAttr[i].enType           = pastSampleVdec[i].enType;
        stChnAttr[i].enMode           = pastSampleVdec[i].enMode;
        stChnAttr[i].u32PicWidth      = pastSampleVdec[i].u32Width;
        stChnAttr[i].u32PicHeight      = pastSampleVdec[i].u32Height;
        stChnAttr[i].u32StreamBufSize = pastSampleVdec[i].u32Width*pastSampleVdec[i].u32Height;
        stChnAttr[i].u32FrameBufCnt   = pastSampleVdec[i].u32FrameBufCnt;

        if (PT_H264 == pastSampleVdec[i].enType || PT_H265 == pastSampleVdec[i].enType)
        {
            stChnAttr[i].stVdecVideoAttr.u32RefFrameNum     = pastSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum;
            stChnAttr[i].stVdecVideoAttr.bTemporalMvpEnable = 1;
            if ((PT_H264 == pastSampleVdec[i].enType) && (VIDEO_DEC_MODE_IPB != pastSampleVdec[i].stSapmleVdecVideo.enDecMode))
            {
                stChnAttr[i].stVdecVideoAttr.bTemporalMvpEnable = 0;
            }
            stChnAttr[i].u32FrameBufSize  = VDEC_GetPicBufferSize(stChnAttr[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height,
                    PIXEL_FORMAT_YVU_SEMIPLANAR_420, pastSampleVdec[i].stSapmleVdecVideo.enBitWidth, 0);

            stChnAttr[i].stVdecVideoAttr.u32TmvBufSize = VDEC_GetTmvBufferSize(stChnAttr[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height);
        }
        else if (PT_JPEG == pastSampleVdec[i].enType || PT_MJPEG == pastSampleVdec[i].enType)
        {
            stChnAttr[i].enMode           = VIDEO_MODE_FRAME;
            stChnAttr[i].u32FrameBufSize  = VDEC_GetPicBufferSize(stChnAttr[i].enType, pastSampleVdec[i].u32Width, pastSampleVdec[i].u32Height,
                                                pastSampleVdec[i].stSapmleVdecPicture.enPixelFormat, DATA_BITWIDTH_8, 0);
        }

        CHECK_CHN_RET(HI_MPI_VDEC_CreateChn(i, &stChnAttr[i]), i, "HI_MPI_VDEC_CreateChn");

        if (VB_SOURCE_USER == g_enVdecVBSource)
        {
            stPool.hPicVbPool = g_ahPicVbPool[i];
            stPool.hTmvVbPool = g_ahTmvVbPool[i];
            CHECK_CHN_RET(HI_MPI_VDEC_AttachVbPool(i, &stPool), i, "HI_MPI_VDEC_AttachVbPool");
        }

        CHECK_CHN_RET(HI_MPI_VDEC_GetChnParam(i, &stChnParam), i, "HI_MPI_VDEC_GetChnParam");
        if (PT_H264 == pastSampleVdec[i].enType || PT_H265 == pastSampleVdec[i].enType)
        {
            stChnParam.stVdecVideoParam.enDecMode         = pastSampleVdec[i].stSapmleVdecVideo.enDecMode;
            stChnParam.stVdecVideoParam.enCompressMode    = COMPRESS_MODE_TILE;
            stChnParam.stVdecVideoParam.enVideoFormat     = VIDEO_FORMAT_TILE_64x16;
            if(VIDEO_DEC_MODE_IPB == stChnParam.stVdecVideoParam.enDecMode)
            {
                stChnParam.stVdecVideoParam.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
            }
            else
            {
                stChnParam.stVdecVideoParam.enOutputOrder = VIDEO_OUTPUT_ORDER_DEC;
            }
        }
        else
        {
            stChnParam.stVdecPictureParam.enPixelFormat   = pastSampleVdec[i].stSapmleVdecPicture.enPixelFormat;
            stChnParam.stVdecPictureParam.u32Alpha        = pastSampleVdec[i].stSapmleVdecPicture.u32Alpha;
        }
        stChnParam.u32DisplayFrameNum                     = pastSampleVdec[i].u32DisplayFrameNum;
        CHECK_CHN_RET(HI_MPI_VDEC_SetChnParam(i, &stChnParam), i, "HI_MPI_VDEC_GetChnParam");

        CHECK_CHN_RET(HI_MPI_VDEC_StartRecvStream(i), i, "HI_MPI_VDEC_StartRecvStream");
    }

    return HI_SUCCESS;
}

static HI_S32 SAMPLE_COMM_VDEC_Stop(HI_S32 s32ChnNum)
{
    HI_S32 i;

    for(i=0; i<s32ChnNum; i++)
    {
        CHECK_CHN_RET(HI_MPI_VDEC_StopRecvStream(i), i, "HI_MPI_VDEC_StopRecvStream");
        CHECK_CHN_RET(HI_MPI_VDEC_DestroyChn(i), i, "HI_MPI_VDEC_DestroyChn");
    }

    return HI_SUCCESS;
}

static HI_VOID SAMPLE_COMM_VDEC_ExitVBPool(HI_VOID)
{
    HI_S32 i, s32Ret;

    if(VB_SOURCE_MODULE == g_enVdecVBSource)
    {
        HI_MPI_VB_ExitModCommPool(VB_UID_VDEC);
    }
    else if (VB_SOURCE_USER == g_enVdecVBSource)
    {
        for (i=VB_MAX_POOLS-1; i>=0; i--)
        {
            if (VB_INVALID_POOLID != g_ahPicVbPool[i])
            {
                s32Ret = HI_MPI_VB_DestroyPool(g_ahPicVbPool[i]);
                if(HI_SUCCESS != s32Ret)
                {
                    printf("HI_MPI_VB_DestroyPool %d fail!\n",g_ahPicVbPool[i]);
                }
                g_ahPicVbPool[i] = VB_INVALID_POOLID;
            }
            if (VB_INVALID_POOLID != g_ahTmvVbPool[i])
            {
                s32Ret = HI_MPI_VB_DestroyPool(g_ahTmvVbPool[i]);
                if(HI_SUCCESS != s32Ret)
                {
                    printf("HI_MPI_VB_DestroyPool %d fail!\n",g_ahTmvVbPool[i]);
                }
                g_ahTmvVbPool[i] = VB_INVALID_POOLID;
            }
        }
    }

    return;
}

static HI_VOID SAMPLE_COMM_SYS_Exit(void)
{
    HI_MPI_SYS_Exit();
    HI_MPI_VB_ExitModCommPool(VB_UID_VDEC);
    HI_MPI_VB_Exit();
    return;
}

static void HIMPP_COMM_VDEC_CopyImage(AVFrame* avframe, VIDEO_FRAME_S* pVBuf)
{
    HI_U8* pY_map = NULL;
    //HI_U8* pC_map = NULL;
    HI_U64 phy_addr;
    HI_U32 u32Size, s32Ysize;

    s32Ysize = (pVBuf->u64PhyAddr[1] - pVBuf->u64PhyAddr[0]);
    u32Size = s32Ysize*3/2;

    phy_addr = pVBuf->u64PhyAddr[0];

    pY_map = (HI_U8*) HI_MPI_SYS_Mmap(phy_addr, u32Size);
    if (HI_NULL == pY_map)
    {
        SAMPLE_PRT("HI_MPI_SYS_Mmap for pY_map fail!!\n");
        return;
    }

    //pC_map = pY_map + s32Ysize;

    /*
    NV21ToI420(pY_map, pVBuf->u32Stride[0], 
        pC_map, pVBuf->u32Stride[1], 
        avframe->data[0], avframe->linesize[0], 
        avframe->data[1], avframe->linesize[1],
        avframe->data[2], avframe->linesize[2],
        avframe->width, avframe->height);
        */

    HI_MPI_SYS_Munmap(pY_map, u32Size);
    pY_map = HI_NULL;

    return;
}

static av_cold int hi_decode_close(AVCodecContext *avctx)
{
  HiContext *ctx = avctx->priv_data;
  
  SAMPLE_COMM_VDEC_Stop(ctx->u32VdecChnNum);

  SAMPLE_COMM_VDEC_ExitVBPool();

  SAMPLE_COMM_SYS_Exit();

  printf("himpp_decode_close\n");

  return 0;
}

static av_cold int hi_decode_init(AVCodecContext *avctx)
{
    HiContext *ctx = avctx->priv_data;
    
    HI_S32 s32Ret = HI_SUCCESS;
    ctx->u32VdecChnNum = 1;

    av_log(avctx, AV_LOG_DEBUG, "video width:%d, height:%d", avctx->width, avctx->height);

    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/
    memset(&ctx->stVbConfig, 0, sizeof(VB_CONFIG_S));
    ctx->stVbConfig.u32MaxPoolCnt             = ctx->u32VdecChnNum;
    ctx->stVbConfig.astCommPool[0].u32BlkCnt  = 10 * ctx->u32VdecChnNum;
    ctx->stVbConfig.astCommPool[0].u64BlkSize = COMMON_GetPicBufferSize(avctx->width, avctx->height,
                                                PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8, COMPRESS_MODE_SEG, 0);
    s32Ret = SAMPLE_COMM_SYS_Init(&ctx->stVbConfig);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "init sys fail for %#x!", s32Ret);
        goto END1;
    }

    /************************************************
    step2:  init module VB or user VB(for VDEC)
    *************************************************/
    ctx->stHiMppVdec.enType                           = PT_H264;
    ctx->stHiMppVdec.u32Width                         = avctx->width;
    ctx->stHiMppVdec.u32Height                        = avctx->height;
    ctx->stHiMppVdec.enMode                           = VIDEO_MODE_FRAME;
    ctx->stHiMppVdec.stSapmleVdecVideo.enDecMode       = VIDEO_DEC_MODE_IP;
    ctx->stHiMppVdec.stSapmleVdecVideo.enBitWidth      = DATA_BITWIDTH_8;
    ctx->stHiMppVdec.stSapmleVdecVideo.u32RefFrameNum  = 3;
    ctx->stHiMppVdec.u32DisplayFrameNum               = 2;
    ctx->stHiMppVdec.u32FrameBufCnt = ctx->stHiMppVdec.stSapmleVdecVideo.u32RefFrameNum + ctx->stHiMppVdec.u32DisplayFrameNum + 1;

    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(ctx->u32VdecChnNum, &ctx->stHiMppVdec);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(ctx->u32VdecChnNum, &ctx->stHiMppVdec);
    if(s32Ret != HI_SUCCESS)
    {
        av_log(avctx, AV_LOG_DEBUG, "start VDEC fail for %#x!\n", s32Ret);
        goto END3;
    }

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    return 0;

  END3:
    SAMPLE_COMM_VDEC_Stop(ctx->u32VdecChnNum);

  END2:
    SAMPLE_COMM_VDEC_ExitVBPool();

  END1:
    SAMPLE_COMM_SYS_Exit();

    return 0;
}

static int hi_decode_frame(AVCodecContext *avctx, void *data, 
                          int *got_frame, AVPacket *avpkt)
{
    struct timeval start_time;

    int ret = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    VGS_HANDLE             hHandle;

    HI_U32                 u32OutWidth;
    HI_U32                 u32OutHeight;

    HI_U32                 u32Align = 0;
    HI_U32                 u32Width;
    HI_U32                 u32Height;
    PIXEL_FORMAT_E         enPixelFormat;
    DATA_BITWIDTH_E        enBitWidth;
    COMPRESS_MODE_E        enCmpMode;
    VB_CAL_CONFIG_S        stCalConfig;

    VB_BLK                 VbHandle = 0;

    struct timeval start_time3;
    struct timeval end_time;

    AVFrame *avframe = data;
    VDEC_STREAM_S stStream;
    VGS_TASK_ATTR_S        stTask;

    uint64_t duration1, duration2, duration3, duration;

    gettimeofday(&start_time, NULL);

    /*
    printf("hi_decode_frame: width:%d, height:%d, pts:%d, size:%u\n", 
           avctx->width, avctx->height, avpkt->pts, avpkt->size);
    */
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
      return 0;
    }

    memset(&stTask, 0, sizeof(stTask));
 
    s32Ret=HI_MPI_VDEC_GetFrame(0, &stTask.stImgIn, 2000);
    if ((HI_SUCCESS != s32Ret))
    {
      av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VDEC_GetFrame fail for %#x!\n", s32Ret);
      return 0;
    }

    /*
    struct timeval start_time1;
    gettimeofday(&start_time1, NULL);
    */
    /************************************************
    Create VGS job
    *************************************************/
    u32OutWidth        = avctx->width;
    u32OutHeight       = avctx->height;
    u32Align = 0;
    u32Width = u32OutWidth;
    u32Height = u32OutHeight;
    enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    enBitWidth = DATA_BITWIDTH_8;
    enCmpMode = COMPRESS_MODE_NONE;
    VbHandle = 0;

    COMMON_GetPicBufferConfig(u32Width, u32Height, enPixelFormat, enBitWidth, enCmpMode, u32Align, &stCalConfig);

    VbHandle = HI_MPI_VB_GetBlock(VB_INVALID_POOLID, stCalConfig.u32VBSize, HI_NULL);

    if (VB_INVALID_HANDLE == VbHandle)
    {
      av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VB_GetBlock failed!\n");
      return 0;
    }

    stTask.stImgOut.u32PoolId                    = HI_MPI_VB_Handle2PoolId(VbHandle);
    stTask.stImgOut.enModId                      = HI_ID_VGS;
    stTask.stImgOut.stVFrame.u32Width            = u32Width;
    stTask.stImgOut.stVFrame.u32Height           = u32Height;
    stTask.stImgOut.stVFrame.enDynamicRange      = DYNAMIC_RANGE_SDR8;
    stTask.stImgOut.stVFrame.enCompressMode      = enCmpMode;
    stTask.stImgOut.stVFrame.enPixelFormat       = enPixelFormat;
    stTask.stImgOut.stVFrame.enVideoFormat       = VIDEO_FORMAT_LINEAR;
    stTask.stImgOut.stVFrame.enField             = VIDEO_FIELD_FRAME;
    stTask.stImgOut.stVFrame.enColorGamut        = COLOR_GAMUT_BT709;
    stTask.stImgOut.stVFrame.u32MaxLuminance     = 1200;
    stTask.stImgOut.stVFrame.u32MinLuminance     = 200;

    stTask.stImgOut.stVFrame.u64HeaderPhyAddr[0] = HI_MPI_VB_Handle2PhysAddr(VbHandle);
    stTask.stImgOut.stVFrame.u64HeaderPhyAddr[1] = stTask.stImgOut.stVFrame.u64HeaderPhyAddr[0] + stCalConfig.u32HeadYSize;
    stTask.stImgOut.stVFrame.u64HeaderPhyAddr[2] = stTask.stImgOut.stVFrame.u64HeaderPhyAddr[1];
    stTask.stImgOut.stVFrame.u64HeaderVirAddr[0] = (HI_U64)HI_MPI_SYS_Mmap(stTask.stImgOut.stVFrame.u64HeaderPhyAddr[0], stCalConfig.u32VBSize);
    stTask.stImgOut.stVFrame.u64HeaderVirAddr[1] = stTask.stImgOut.stVFrame.u64HeaderVirAddr[0] + stCalConfig.u32HeadYSize;
    stTask.stImgOut.stVFrame.u64HeaderVirAddr[2] = stTask.stImgOut.stVFrame.u64HeaderVirAddr[1];
    stTask.stImgOut.stVFrame.u32HeaderStride[0]  = stCalConfig.u32HeadStride;
    stTask.stImgOut.stVFrame.u32HeaderStride[1]  = stCalConfig.u32HeadStride;
    stTask.stImgOut.stVFrame.u32HeaderStride[2]  = stCalConfig.u32HeadStride;

    stTask.stImgOut.stVFrame.u64PhyAddr[0]       = stTask.stImgOut.stVFrame.u64HeaderPhyAddr[0] + stCalConfig.u32HeadSize;
    stTask.stImgOut.stVFrame.u64PhyAddr[1]       = stTask.stImgOut.stVFrame.u64PhyAddr[0] + stCalConfig.u32MainYSize;
    stTask.stImgOut.stVFrame.u64PhyAddr[2]       = stTask.stImgOut.stVFrame.u64PhyAddr[1];
    stTask.stImgOut.stVFrame.u64VirAddr[0]       = stTask.stImgOut.stVFrame.u64HeaderVirAddr[0] + stCalConfig.u32HeadSize;
    stTask.stImgOut.stVFrame.u64VirAddr[1]       = stTask.stImgOut.stVFrame.u64VirAddr[0] + stCalConfig.u32MainYSize;
    stTask.stImgOut.stVFrame.u64VirAddr[2]       = stTask.stImgOut.stVFrame.u64VirAddr[1];
    stTask.stImgOut.stVFrame.u32Stride[0]        = stCalConfig.u32MainStride;
    stTask.stImgOut.stVFrame.u32Stride[1]        = stCalConfig.u32MainStride;
    stTask.stImgOut.stVFrame.u32Stride[2]        = stCalConfig.u32MainStride;

    stTask.stImgOut.stVFrame.u64ExtPhyAddr[0]    = stTask.stImgOut.stVFrame.u64PhyAddr[0] + stCalConfig.u32MainSize;
    stTask.stImgOut.stVFrame.u64ExtPhyAddr[1]    = stTask.stImgOut.stVFrame.u64ExtPhyAddr[0] + stCalConfig.u32ExtYSize;
    stTask.stImgOut.stVFrame.u64ExtPhyAddr[2]    = stTask.stImgOut.stVFrame.u64ExtPhyAddr[1];
    stTask.stImgOut.stVFrame.u64ExtVirAddr[0]    = stTask.stImgOut.stVFrame.u64VirAddr[0] + stCalConfig.u32MainSize;
    stTask.stImgOut.stVFrame.u64ExtVirAddr[1]    = stTask.stImgOut.stVFrame.u64ExtVirAddr[0] + stCalConfig.u32ExtYSize;
    stTask.stImgOut.stVFrame.u64ExtVirAddr[2]    = stTask.stImgOut.stVFrame.u64ExtVirAddr[1];
    stTask.stImgOut.stVFrame.u32ExtStride[0]     = stCalConfig.u32ExtStride;
    stTask.stImgOut.stVFrame.u32ExtStride[1]     = stCalConfig.u32ExtStride;
    stTask.stImgOut.stVFrame.u32ExtStride[2]     = stCalConfig.u32ExtStride;

    s32Ret = HI_MPI_VGS_BeginJob(&hHandle);
    if(HI_SUCCESS != s32Ret)
    {
      av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VGS_BeginJob failed, s32Ret:0x%x\n",s32Ret);
      goto EXIT2;
    }

    s32Ret = HI_MPI_VGS_AddScaleTask(hHandle, &stTask, VGS_SCLCOEF_NORMAL);
    if(HI_SUCCESS != s32Ret)
    {
      av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VGS_AddScaleTask failed, s32Ret:0x%x\n",s32Ret);
      goto EXIT2;
    }

    s32Ret = HI_MPI_VGS_EndJob(hHandle);
    if(HI_SUCCESS != s32Ret)
    {
      av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VGS_EndJob failed, s32Ret:0x%x\n",s32Ret);
      goto EXIT2;
    }

    /*
    struct timeval start_time2;
    gettimeofday(&start_time2, NULL);
    */
    ret = ff_set_dimensions(avctx, stTask.stImgOut.stVFrame.u32Width, stTask.stImgOut.stVFrame.u32Height);
    if (ret < 0)
        goto EXIT2;

    // The decoder doesn't (currently) support decoding into a user
    // provided buffer, so do a copy instead.
    if (ff_get_buffer(avctx, avframe, 0) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Unable to allocate buffer\n");
        //return AVERROR(ENOMEM);
        goto EXIT2;
    }

    HIMPP_COMM_VDEC_CopyImage(avframe, &stTask.stImgOut.stVFrame);

    gettimeofday(&start_time3, NULL);

    /************************************************
    Munmap
    *************************************************/
    s32Ret = HI_MPI_SYS_Munmap((HI_VOID*)stTask.stImgOut.stVFrame.u64HeaderVirAddr[0],stCalConfig.u32VBSize);

    if(HI_SUCCESS != s32Ret)
    {
        av_log(avctx, AV_LOG_DEBUG, "HI_MPI_SYS_Munmap failed, s32Ret:0x%x\n",s32Ret);
        goto EXIT1;
    }

    /************************************************
    Release VB
    *************************************************/
    s32Ret = HI_MPI_VB_ReleaseBlock(VbHandle);

    if(HI_SUCCESS != s32Ret)
    {
        av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VB_ReleaseBlock failed, s32Ret:0x%x\n",s32Ret);
        goto EXIT;
    }

    //printf("pts:%"PRIu64"\n", stTask.stImgOut.stVFrame.u64PTS);
    avframe->pts     = stTask.stImgOut.stVFrame.u64PTS;
    avframe->pkt_dts = AV_NOPTS_VALUE;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
    avframe->pkt_pts = avpkt->pts;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
    
    *got_frame = 1;
    ret = avpkt->size;

    /*
    FILE* fd = fopen("a.yuv", "ab");
    
    uint8_t* y = avframe->data[0]; 
    for (int i = 0; i < avframe->height; i++) {
      fwrite(y, 1, avframe->width, fd);
      y += avframe->width;
    }

    uint8_t* u = avframe->data[1];
    for (int i = 0; i < avframe->height / 2; i++) {
      fwrite(u, 1, avframe->width / 2, fd);
      u += avframe->width / 2;
    }

    uint8_t* v = avframe->data[2];
    for (int i = 0; i < avframe->height / 2; i++) {
      fwrite(v, 1, avframe->width / 2, fd);
      v += avframe->width / 2;
    }

    fclose(fd);
    */

  EXIT2:
    HI_MPI_SYS_Munmap((HI_VOID*)stTask.stImgOut.stVFrame.u64HeaderVirAddr[0],stCalConfig.u32VBSize);
  EXIT1:
    HI_MPI_VB_ReleaseBlock(VbHandle);
  EXIT:       
    s32Ret=HI_MPI_VDEC_ReleaseFrame(0, &stTask.stImgIn);
    if (HI_SUCCESS != s32Ret)
    {
        av_log(avctx, AV_LOG_DEBUG, "HI_MPI_VDEC_ReleaseFrame fail for s32Ret=0x%x!\n", s32Ret);
    }

    gettimeofday(&end_time, NULL);

    duration1 = 0; //(start_time1.tv_sec * 1000 + start_time1.tv_usec / 1000) - (start_time.tv_sec * 1000 + start_time.tv_usec / 1000);   

    duration2 = 0; //= (start_time2.tv_sec * 1000 + start_time2.tv_usec / 1000) - (start_time.tv_sec * 1000 + start_time.tv_usec / 1000); 

    duration3 = (start_time3.tv_sec * 1000 + start_time3.tv_usec / 1000) - (start_time.tv_sec * 1000 + start_time.tv_usec / 1000); 

    duration = (end_time.tv_sec * 1000 + end_time.tv_usec / 1000) - (start_time.tv_sec * 1000 + start_time.tv_usec / 1000);
    av_log(avctx, AV_LOG_DEBUG, "got_frame:%d, ret:%d, duration1:%"PRIu64"ms duration2:%"PRIu64"ms duration3:%"PRIu64"ms duration:%"PRIu64"ms\n", 
        *got_frame, ret, duration1, duration2, duration3, duration);

    return ret;
}

AVCodec ff_h264_himpp_decoder = {
    .name           = "h264_himpp",
    .long_name      = NULL_IF_CONFIG_SMALL("hisi mpp H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H264,
    .priv_data_size = sizeof(HiContext),
    .init           = hi_decode_init,
    .decode         = hi_decode_frame,
    .close          = hi_decode_close,
    .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_SETS_PKT_DTS | FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
    .bsfs           = "h264_mp4toannexb",
    .wrapper_name   = "himpp",
};
