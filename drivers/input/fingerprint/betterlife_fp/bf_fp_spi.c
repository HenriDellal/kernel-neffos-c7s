#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/clk.h>
#include <linux/version.h>

#if defined(PLATFORM_MT6739) || defined(PLATFORM_MTK)
#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#endif

#endif

#include "bf_fp_platform.h"
/* MTK header */
#if defined(PLATFORM_MT6739)
#include <linux/platform_data/spi-mt65xx.h>
static struct mtk_chip_config spi_init_conf = {
	.cs_pol = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.sample_sel = 0,
};
#elif defined(PLATFORM_MTK)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 0, 0))
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#include "mtk_gpio.h"
#include "mach/gpio_const.h"
#else
#include "mt_spi.h"
#include <mt_chip.h>
#endif

static struct mt_chip_conf spi_init_conf = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 8,
	.low_time =  8,
	.cs_idletime = 20, //10,
	//.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

extern struct bf_device *g_bf_dev;
u32 g_chip_type = 0;
static DEFINE_MUTEX(spi_lock);

int mtspi_set_dma_en(int mode)
{
#if defined(PLATFORM_MTK)
	struct mt_chip_conf* spi_par;
	spi_par = &spi_init_conf;
	if (!spi_par) {
		return -1;
	}
	if (1 == mode) {
		if (spi_par->com_mod == DMA_TRANSFER) {
			return 0;
		}
		spi_par->com_mod = DMA_TRANSFER;
	} else {
		if (spi_par->com_mod == FIFO_TRANSFER) {
			return 0;
		}
		spi_par->com_mod = FIFO_TRANSFER;
	}
	spi_setup(g_bf_dev->spi);
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
int spi_send_cmd(struct bf_device *bl229x,u8 *tx,u8 *rx,u16 spilen)
{
#ifdef TEE_TK
    extern int tee_spi_transfer(void *conf, uint32_t conf_size, void *inbuf, void *outbuf, uint32_t size);
	int ret = tee_spi_transfer(&spi_init_conf, sizeof(spi_init_conf), tx, rx, spilen);
#else
	int ret=0;
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
//#if USE_SPI1_4GB_TEST
//		.tx_dma = SpiDmaBufTx_pa,
//		.rx_dma = SpiDmaBufRx_pa,
//#endif
		.tx_dma = 0,
		.rx_dma = 0,
	};

	mutex_lock(&spi_lock);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret= spi_sync(bl229x->spi,&m);
	mutex_unlock(&spi_lock);
#endif
	return ret;
}

/*----------------------------------------------------------------------------*/
u8 bf_spi_write_reg(u8 reg, u8 value)
{
	u8 nAddr;
	u8 data_tx[4] = {0};
	u8 data_rx[4] = {0};

	nAddr = reg << 1;
	nAddr |= 0x80;

	data_tx[0] = nAddr;
	data_tx[1] = value;

	spi_send_cmd(g_bf_dev,data_tx,data_rx,2);
	return data_rx[1];
}

//-------------------------------------------------------------------------------------------------
u8 bf_spi_read_reg(u8 reg)
{
	u8 nAddr;
	u8 data_tx[4] = {0};
	u8 data_rx[4] = {0};

	nAddr = reg << 1;
	nAddr &= 0x7F;

	data_tx[0] = nAddr;
	data_tx[1] = 0xff;

	spi_send_cmd(g_bf_dev,data_tx,data_rx,2);
	return data_rx[1];
}

	
/*----------------------------------------------------------------------------*/
u8 bf_spi_write_reg_bit(u8 nRegID, u8 bit, u8 value)
{
  u8 tempvalue = 0;

  tempvalue = bf_spi_read_reg(nRegID);
  tempvalue &= ~(1 << bit);
  tempvalue |= (value << bit);
  bf_spi_write_reg(nRegID,tempvalue);

  return 0;
}

int bf_read_chipid(void)
{
    u8 val_low = 0, val_high = 0, version = 0;
    int chip_id = 0;
    u8	reg_value = 0;

    if(g_chip_type == BF3290) {
        bf_spi_write_reg (0x13, 0x00);
        reg_value = bf_spi_read_reg (0x3a);
        bf_spi_write_reg (0x3a, reg_value | 0x80);

        val_low = bf_spi_read_reg (0x10); //id reg low
        BF_LOG ("val_low=0x%x \n", val_low);

        val_high = bf_spi_read_reg (0x11); //id reg high
        BF_LOG ("val_high=0x%x \n", val_high);

        version = bf_spi_read_reg (0x12); //ic type
        BF_LOG ("version=0x%x \n", version);
        chip_id = (val_high << 8) | (val_low & 0xff);
        BF_LOG ("chip_id=%x \n", chip_id);
        bf_spi_write_reg (0x3a, reg_value);
    } else if(g_chip_type == BF3182 || g_chip_type == BF3390) {
        //enable 0x10 bit5
        reg_value = bf_spi_read_reg(0x10);
        reg_value &= ~(1 << 5);
        bf_spi_write_reg(0x10, reg_value);

        val_high = bf_spi_read_reg(0x37);
        val_low = bf_spi_read_reg(0x36);
        chip_id = (val_high << 8) | val_low;
        version = bf_spi_read_reg(0x38);

        //disabl 0x10 bit5
        reg_value |= (1 << 5);
        bf_spi_write_reg(0x10, reg_value);
    } else {
        val_high = bf_spi_read_reg(0x37);
        val_low = bf_spi_read_reg(0x36);
        chip_id = (val_high << 8) | val_low;
        version = bf_spi_read_reg(0x38);

        if(chip_id != BF3582P && chip_id != BF3582S) {
            //enable 0x10 bit5
            reg_value = bf_spi_read_reg(0x10);
            reg_value &= ~(1 << 5);
            bf_spi_write_reg(0x10, reg_value);

            val_high = bf_spi_read_reg(0x37);
            val_low = bf_spi_read_reg(0x36);
            chip_id = (val_high << 8) | val_low;
            version = bf_spi_read_reg(0x38);

            //disabl 0x10 bit5
            reg_value |= (1 << 5);
            bf_spi_write_reg(0x10, reg_value);
            if(chip_id != BF3182 && chip_id != BF3390) {
                bf_spi_write_reg (0x13, 0x00);
                reg_value = bf_spi_read_reg (0x3a);
                bf_spi_write_reg (0x3a, reg_value | 0x80);

                val_low = bf_spi_read_reg (0x10); //id reg low
                BF_LOG ("val_low=0x%x \n", val_low);

                val_high = bf_spi_read_reg (0x11); //id reg high
                BF_LOG ("val_high=0x%x \n", val_high);

                version = bf_spi_read_reg (0x12); //ic type
                BF_LOG ("version=0x%x \n", version);
                chip_id = (val_high << 8) | (val_low & 0xff);
                BF_LOG ("chip_id=%x \n", chip_id);
                bf_spi_write_reg (0x3a, reg_value);
            }
        }
    }

    BF_LOG(" chip_id=0x%x,version=0x%x\n", chip_id, version);
    return chip_id;
}

void bf_chip_info(void)
{
    BF_LOG("data:2017-12-18\n");
    g_chip_type = bf_read_chipid();
    BF_LOG("BTL: chipid:%x\r\n", g_chip_type);
}

#ifdef CONFIG_MTK_CLK
void bf_spi_clk_enable(struct bf_device *bf_dev, u8 onoff)
{
	static int count = 0;

#if defined(PLATFORM_MT6739)
	struct mtk_spi *mdata = NULL;
	int ret = -1;
	mdata = spi_master_get_devdata(bf_dev->spi->master);

	if (onoff && (count == 0)) {
		ret = clk_prepare_enable(mdata->spi_clk);
		if (ret < 0) {
			BF_LOG("failed to enable spi_clk (%d)\n", ret);
		}else
			count = 1;
	} else if ((count > 0) && (onoff == 0)) {
		clk_disable_unprepare(mdata->spi_clk);
		count = 0;
	}
#elif defined(PLATFORM_MTK)
    if (onoff && (count == 0)) {
        mt_spi_enable_master_clk(bf_dev->spi);
        count = 1;
    } else if ((count > 0) && (onoff == 0)) {
        mt_spi_disable_master_clk(bf_dev->spi);
        count = 0;
    }
#endif
}
#endif

#if !defined(PLATFORM_SPRD) && !defined(PLATFORM_QCOM)
static int32_t bf_platform_pinctrl_spi_init(struct bf_device *bf_dev)
{
	int32_t error = 0;
	
#ifdef BF_PINCTL
	error = pinctrl_select_state(bf_dev->pinctrl_gpios, bf_dev->pins_spi_default);
	if (error) 
	{
		dev_err(&bf_dev->pdev->dev, "failed to activate pins_spi_default state\n");
	}
#endif
	return error;
}

static int32_t bf_platform_gpio_spi_init(struct bf_device *bf_dev)
{
	return 0;
}
#endif

static int bf_spi_suspend (struct device *dev)
{
    return 0;
}

static int bf_spi_resume (struct device *dev)
{
    return 0;
}

static int bf_spi_remove(struct spi_device *spi)
{
    return 0;
}

static int bf_spi_probe (struct spi_device *spi)
{
	int status = -EINVAL;
#if !defined(PLATFORM_SPRD) && !defined(PLATFORM_QCOM)
	/* Initialize the driver data */
	BF_LOG( "bf config spi ");
	g_bf_dev->spi = spi;
	/* setup SPI parameters */
	g_bf_dev->spi->mode = SPI_MODE_0;
	g_bf_dev->spi->bits_per_word = 8;
	g_bf_dev->spi->max_speed_hz = 6 * 1000 * 1000;

	g_bf_dev->spi->controller_data = (void*)&spi_init_conf;
	spi_setup (g_bf_dev->spi);    
	spi_set_drvdata (spi, g_bf_dev);

	BF_LOG ("---ok--");
	return 0;

err:	
	return status;
#else
	return status;
#endif
}

static const struct dev_pm_ops bf_pm =
{
    .suspend = bf_spi_suspend,
    .resume =  bf_spi_resume
};

#ifdef CONFIG_OF
static struct of_device_id bf_of_spi_table[] = {
	{.compatible = "betterlife,spi",},
	{},
};
MODULE_DEVICE_TABLE(of, bf_of_table);
#endif

//static struct spi_device_id bf_spi_device_id = {};
static struct spi_driver bf_spi_driver = {
    .driver = {
        .name = BF_DEV_NAME,
		.bus	= &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bf_of_spi_table,
#endif
		.pm = &bf_pm,
    },
    .probe = bf_spi_probe,
    .remove = bf_spi_remove,
};

int32_t bf_spi_init(struct bf_device *bf_dev)
{
    int32_t error = 0;

#if !defined(PLATFORM_SPRD) && !defined(PLATFORM_QCOM)
#if defined(BF_PINCTL)
	error = bf_platform_pinctrl_spi_init(bf_dev);
	if(error)
	{
		BF_LOG("bf_platform_pinctrl_init fail");
		goto out;
	}
#else /*else BF_PINCTRL*/
	error = bf_platform_gpio_spi_init(bf_dev);
	if(error)
	{
		BF_LOG("bf_platform_gpio_init fail");
		goto out;
	}
#endif	//BF_PINCTL

	spi_register_driver(&bf_spi_driver);

out:
	return error;
#else
        spi_register_driver(&bf_spi_driver);
	return error;
#endif
}

int32_t bf_platform_uninit(struct bf_device *bf_dev)
{
#ifdef CONFIG_MTK_CLK
	bf_spi_clk_enable(bf_dev,0);
#endif

	return 0;
}

int32_t bf_spi_check_chipid(struct bf_device *bf_dev)
{
    int32_t status = 0;
    status = bf_spi_init(bf_dev);
    if(status)
    {
        BF_LOG("bf_platform_gpio_init fail:%d", status);
        goto err;
    }

#ifdef CONFIG_MTK_CLK
	bf_spi_clk_enable(bf_dev,1);
#endif

	bf_chip_info();
	if(g_chip_type !=0x5183 && g_chip_type!=0x5283 && g_chip_type!=0x5383 && g_chip_type!=0x5483 && g_chip_type!=0x5783){
		BF_LOG("ChipID:0x%x error and exit", g_chip_type);
		bf_platform_uninit(bf_dev);
		return -1;
	}
	
#if defined(TEE_BEANPOD) && defined(COMPATIBLE)
	//set_fp_vendor(FP_VENDOR_BETTERLIFE);
#endif

/*#ifdef CONFIG_MTK_CLK
	bf_spi_clk_enable(bf_dev,0);
#endif*/

	return 0;
err:
    BF_LOG("bf_platform_gpio_init fail");
	bf_platform_uninit(bf_dev);
    return -1;
}

