#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#ifdef ARCH_CV182X
#include <linux/cvi_vip_snsr.h>
#include "cvi_comm_video.h"
#else
#include <linux/vi_snsr.h>
#include <linux/cvi_comm_video.h>
#endif
#include "cvi_sns_ctrl.h"
#include "ov5647_cmos_ex.h"

static void ov5647_linear_1080p30_init(VI_PIPE ViPipe);

CVI_U8 ov5647_i2c_addr = 0x36;        /* I2C Address of OV5647 */
const CVI_U32 ov5647_addr_byte = 2;
const CVI_U32 ov5647_data_byte = 1;
static int g_fd[VI_MAX_PIPE_NUM] = {[0 ... (VI_MAX_PIPE_NUM - 1)] = -1};

int ov5647_i2c_init(VI_PIPE ViPipe)
{

	char acDevFile[16] = {0};
	CVI_U8 u8DevNum;

	if (g_fd[ViPipe] >= 0)
		return CVI_SUCCESS;
	int ret;

	u8DevNum = g_aunOv5647_BusInfo[ViPipe].s8I2cDev;
	snprintf(acDevFile, sizeof(acDevFile),  "/dev/i2c-%u", u8DevNum);

	g_fd[ViPipe] = open(acDevFile, O_RDWR, 0600);

	if (g_fd[ViPipe] < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Open /dev/i2c-%u error!\n", u8DevNum);
		return CVI_FAILURE;
	}

	ret = ioctl(g_fd[ViPipe], I2C_SLAVE_FORCE, ov5647_i2c_addr);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_SLAVE_FORCE error!\n");
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return ret;
	}
	return CVI_SUCCESS;
}

int ov5647_i2c_exit(VI_PIPE ViPipe)
{
	if (g_fd[ViPipe] >= 0) {
		close(g_fd[ViPipe]);
		g_fd[ViPipe] = -1;
		return CVI_SUCCESS;
	}
	return CVI_FAILURE;
}

int ov5647_read_register(VI_PIPE ViPipe, int addr)
{
	int ret, data;
	CVI_U8 buf[8];
	CVI_U8 idx = 0;

	if (g_fd[ViPipe] < 0)
		return CVI_FAILURE;

	if (ov5647_addr_byte == 2)
		buf[idx++] = (addr >> 8) & 0xff;

	// add address byte 0
	buf[idx++] = addr & 0xff;

	ret = write(g_fd[ViPipe], buf, ov5647_addr_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_READ error!\n");
		return 0;
	}

	buf[0] = 0;
	buf[1] = 0;
	ret = read(g_fd[ViPipe], buf, ov5647_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_READ error!\n");
		return 0;
	}

	// pack read back data
	data = 0;
	if (ov5647_data_byte == 2) {
		data = buf[0] << 8;
		data += buf[1];
	} else {
		data = buf[0];
	}

	// syslog(LOG_DEBUG, "i2c r 0x%x = 0x%x\n", addr, data);

	return data;
}

int ov5647_write_register(VI_PIPE ViPipe, int addr, int data)
{
	CVI_U8 idx = 0;
	int ret;
	CVI_U8 buf[8];

	if (g_fd[ViPipe] < 0)
		return CVI_SUCCESS;

	if (ov5647_addr_byte == 2) {
		buf[idx] = (addr >> 8) & 0xff;
		idx++;
		buf[idx] = addr & 0xff;
		idx++;
	}

	if (ov5647_data_byte == 1) {
		buf[idx] = data & 0xff;
		idx++;
	}

	ret = write(g_fd[ViPipe], buf, ov5647_addr_byte + ov5647_data_byte);
	if (ret < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "I2C_WRITE error!\n");
		return CVI_FAILURE;
	}
	// syslog(LOG_DEBUG, "i2c w 0x%x 0x%x\n", addr, data);
	return CVI_SUCCESS;
}

static void delay_ms(int ms)
{
	usleep(ms * 1000);
}

void ov5647_standby(VI_PIPE ViPipe)
{
	ov5647_write_register(ViPipe, 0x0100, 0x00); /* STANDBY */
}

void ov5647_restart(VI_PIPE ViPipe)
{
	ov5647_write_register(ViPipe, 0x0100, 0x01); /* standby */
}

void ov5647_default_reg_init(VI_PIPE ViPipe)
{
	CVI_U32 i;
	CVI_U32 start = 1;
	CVI_U32 end = g_pastOv5647[ViPipe]->astSyncInfo[0].snsCfg.u32RegNum - 3;

	for (i = start; i < end; i++) {
		ov5647_write_register(ViPipe,
				g_pastOv5647[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32RegAddr,
				g_pastOv5647[ViPipe]->astSyncInfo[0].snsCfg.astI2cData[i].u32Data);
	}
}

#define OV5647_FLIP	0x3820
#define OV5647_MIRROR	0x3821
void ov5647_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip)
{
	CVI_U8 flip, mirror;

	flip = ov5647_read_register(ViPipe, OV5647_FLIP);
	mirror = ov5647_read_register(ViPipe, OV5647_MIRROR);
	flip &= ~(0x3 << 1);
	mirror &= ~(0x3 << 1);

	switch (eSnsMirrorFlip) {
	case ISP_SNS_NORMAL:
		break;
	case ISP_SNS_MIRROR:
		mirror |= 0x3 << 1;
		break;
	case ISP_SNS_FLIP:
		flip |= 0x3 << 1;
		break;
	case ISP_SNS_MIRROR_FLIP:
		flip |= 0x3 << 1;
		mirror |= 0x3 << 1;
		break;
	default:
		return;
	}

	ov5647_write_register(ViPipe, OV5647_FLIP, flip);
	ov5647_write_register(ViPipe, OV5647_MIRROR, mirror);
}

#define OV5647_CHIP_ID_ADDR_H		0x300A
#define OV5647_CHIP_ID_ADDR_L		0x300B
#define OV5647_CHIP_ID			0x5647

int ov5647_probe(VI_PIPE ViPipe)
{
	int nVal, nVal2;

	usleep(1000);
	if (ov5647_i2c_init(ViPipe) != CVI_SUCCESS)
		return CVI_FAILURE;

	nVal  = ov5647_read_register(ViPipe, OV5647_CHIP_ID_ADDR_H);
	nVal2 = ov5647_read_register(ViPipe, OV5647_CHIP_ID_ADDR_L);
	if (nVal < 0 || nVal2 < 0) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "read sensor id error.\n");
		return nVal;
	}

	if ((((nVal & 0xFF) << 8) | (nVal2 & 0xFF)) != OV5647_CHIP_ID) {
		CVI_TRACE_SNS(CVI_DBG_ERR, "Sensor ID Mismatch! Use the wrong sensor??\n");
		return CVI_FAILURE;
	}
	return CVI_SUCCESS;
}

void ov5647_init(VI_PIPE ViPipe)
{
	ov5647_i2c_init(ViPipe);

	delay_ms(10);

	ov5647_linear_1080p30_init(ViPipe);

	g_pastOv5647[ViPipe]->bInit = CVI_TRUE;
}

void ov5647_exit(VI_PIPE ViPipe)
{
	ov5647_i2c_exit(ViPipe);
}

/* 1080P30 (Hackeado a 5MP 2592x1944) */
static void ov5647_linear_1080p30_init(VI_PIPE ViPipe)
{
	int i;
	
	/* Matriz de registros para 5MP extraída del kernel de Raspberry Pi */
	static const struct {
		int addr;
		int data;
	} ov5647_5mp_15fps[] = {
		{0x0100, 0x00}, // Software standby
		{0x0103, 0x01}, // Software reset
		// Relojes y PLL
		{0x3034, 0x1a}, {0x3035, 0x21}, {0x3036, 0x69}, {0x3016, 0x08},
		{0x303c, 0x11}, {0x3106, 0xf5}, {0x3821, 0x00}, {0x3820, 0x00},
		// Geometría 5MP (2592x1944)
		{0x3800, 0x00}, // X start
		{0x3801, 0x00},
		{0x3802, 0x00}, // Y start
		{0x3803, 0x00},
		{0x3804, 0x0a}, // X end (2592)
		{0x3805, 0x3f},
		{0x3806, 0x07}, // Y end (1944)
		{0x3807, 0xa3},
		{0x3808, 0x0a}, // X output size (2592)
		{0x3809, 0x20},
		{0x380a, 0x07}, // Y output size (1944)
		{0x380b, 0x98},
		{0x380c, 0x0b}, // HTS (2844)
		{0x380d, 0x1c},
		{0x380e, 0x07}, // VTS (1968)
		{0x380f, 0xb0},
		{0x3814, 0x11}, // X inc
		{0x3815, 0x11}, // Y inc
		// Ajustes ópticos
		{0x3612, 0x5b}, {0x3618, 0x04}, {0x4004, 0x04}, {0x4005, 0x1a},
		{0x3503, 0x03}, {0x5001, 0x01}, {0x5000, 0x06}, {0x0100, 0x01}, // Wake up
	};

	int array_size = sizeof(ov5647_5mp_15fps) / sizeof(ov5647_5mp_15fps[0]);

	/* Volcar la tabla I2C al sensor */
	for (i = 0; i < array_size; i++) {
		ov5647_write_register(ViPipe, ov5647_5mp_15fps[i].addr, ov5647_5mp_15fps[i].data);
	}

	/* Mantener la inicialización de exposición y ganancia original */
	ov5647_default_reg_init(ViPipe);

	delay_ms(100);

	printf("ViPipe:%d,===OV5647 5MP 15fps HACK Init OK!\n", ViPipe);
}




