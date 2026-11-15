//加速度传感器 LIS2DH12 测试程序
#include <errno.h>
#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>

#define LIS2DH12_REG_WHO_AM_I       0x0F
#define LIS2DH12_REG_CTRL1          0x20
#define LIS2DH12_REG_CTRL4          0x23
#define LIS2DH12_REG_STATUS         0x27
#define LIS2DH12_REG_OUT_X_L        0x28
#define LIS2DH12_WHO_AM_I_VALUE     0x33
#define LIS2DH12_AUTO_INCREMENT     BIT(7)
//CTRL_REG1：100 Hz，X/Y/Z 启用，高分辨率模式。
#define LIS2DH12_CTRL1_ODR_100_HZ   (5U << 4)
#define LIS2DH12_CTRL1_XYZ_ENABLE   (BIT(0) | BIT(1) | BIT(2))
#define LIS2DH12_CTRL1_VALUE        \
	(LIS2DH12_CTRL1_ODR_100_HZ | LIS2DH12_CTRL1_XYZ_ENABLE)
//CTRL_REG4：块数据更新，高分辨率，量程为±2g
#define LIS2DH12_CTRL4_BDU          BIT(7)
#define LIS2DH12_CTRL4_FS_2G        (0U << 4)
#define LIS2DH12_CTRL4_HR           BIT(3)
#define LIS2DH12_CTRL4_VALUE        \
	(LIS2DH12_CTRL4_BDU | LIS2DH12_CTRL4_FS_2G | LIS2DH12_CTRL4_HR)
#define LIS2DH12_STATUS_DRDY_MASK   BIT_MASK(4)
#define LIS2DH12_ACCEL_BURST_SIZE   7U

#define UART_NODE DT_NODELABEL(uart0)
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
uint8_t rx_buf[10]={0};
uint8_t tx_buf[128];

static const struct i2c_dt_spec lis2dh12 =I2C_DT_SPEC_GET(DT_NODELABEL(myaccel));

static void i2c_scan(const struct device *bus)
{
    printk("scanning...\n");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy;
        if (i2c_read(bus, &dummy, 1, addr) == 0) {
            printk("  found 0x%02X\n", addr);
        }
    }
}

void uart_cd(const struct device *dev, 
    struct uart_event *evt, void *user_data)
{
    switch(evt->type)
    {
        case UART_TX_DONE:
            LOG_INF("整个TX缓冲区已传输完毕。");
            break;
        case UART_TX_ABORTED:
            LOG_INF("由于超时或调用了uart_tx_abort，传输被中止");
            break;
        case UART_RX_RDY:
            LOG_INF("已接收的数据已准备好进行处理。");
            LOG_INF("接收到数据时：%s", evt->data.rx.buf);
            break;
        case UART_RX_BUF_REQUEST:
            LOG_INF("驱动程序请求下一个缓冲区以实现连续接收");
            break;
        case UART_RX_BUF_RELEASED:
            LOG_INF(" UART 驱动程序不再使用缓冲区");
            break;
        case  UART_RX_DISABLED: 
            uart_rx_enable(uart_dev,rx_buf,sizeof(rx_buf),100);
            LOG_INF(" RX 已被禁用，可以重新启用。");
            break;
        case  UART_RX_STOPPED:
            LOG_INF("RX 因外部事件而停止运行。");
            break;
        default:
            LOG_INF("Unknown event type: %d", evt->type);
            break;
    }

}

struct lis2dh12_sample {
	uint8_t status;

	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;

	int32_t x_mg;
	int32_t y_mg;
	int32_t z_mg;
};

static int lis2dh12_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t buffer[2] = {reg, value};

	return i2c_write_dt(&lis2dh12, buffer, sizeof(buffer));
}

static int lis2dh12_init(void)
{
	uint8_t id = 0;
	int ret;

	if (!i2c_is_ready_dt(&lis2dh12)) {
		printk("LIS2DH12: I2C bus is not ready\n");
		return -ENODEV;
	}
	//The device needs a few milliseconds after power-on. 
	k_sleep(K_MSEC(10));

	ret = i2c_reg_read_byte_dt(&lis2dh12, LIS2DH12_REG_WHO_AM_I, &id);
	if (ret < 0) {
		printk("LIS2DH12: WHO_AM_I read failed: %d\n", ret);
		return ret;
	}
	if (id != LIS2DH12_WHO_AM_I_VALUE) {
		printk("LIS2DH12: unexpected WHO_AM_I 0x%02X\n", id);
		return -ENODEV;
	}
	// 在更改传感器配置时，请停止传感器
	ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL1, 0);
	if (ret < 0) {
		return ret;
	}

	ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL4, LIS2DH12_CTRL4_VALUE);
	if (ret < 0) {
		return ret;
	}

	ret = lis2dh12_write_reg(LIS2DH12_REG_CTRL1, LIS2DH12_CTRL1_VALUE);
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(20));
	printk("LIS2DH12: initialized, WHO_AM_I=0x%02X\n", id);

	return 0;
}
//在±2 g的高分辨率模式下，12位原始值的标度为
//每计数单位约1000/1024毫克。
static int32_t lis2dh12_raw_to_mg(int16_t raw)
{
	int32_t scaled = (int32_t)raw * 1000;

	return (scaled >= 0) ? ((scaled + 512) / 1024)
			     : ((scaled - 512) / 1024);
}

static void lis2dh12_get_axis(const uint8_t data[LIS2DH12_ACCEL_BURST_SIZE],
			      size_t offset, int16_t *raw, int32_t *mg)
{
	int16_t value;

	value = (int16_t)((uint16_t)data[offset] |
			  ((uint16_t)data[offset + 1] << 8));

	//LIS2DH 数据为左对齐；保留带符号的 12 位值。
	*raw = value >> 4;
	*mg = lis2dh12_raw_to_mg(*raw);
}

static int lis2dh12_read(struct lis2dh12_sample *sample)
{
	uint8_t data[LIS2DH12_ACCEL_BURST_SIZE];
	int ret;
	//
    //在一次自动递增的传输中读取 STATUS 以及随后的 X/Y/Z 的 LSB/MSB。
    //这样可以确保三个轴的数据均来自同一个采样点。
    //
	ret = i2c_burst_read_dt(
		&lis2dh12,
		LIS2DH12_REG_STATUS | LIS2DH12_AUTO_INCREMENT,
		data,
		sizeof(data));

	if (ret < 0) {
		return ret;
	}

	sample->status = data[0];
	if ((sample->status & LIS2DH12_STATUS_DRDY_MASK) == 0U) {
		return -EAGAIN;
	}

	lis2dh12_get_axis(data, 1, &sample->x_raw, &sample->x_mg);
	lis2dh12_get_axis(data, 3, &sample->y_raw, &sample->y_mg);
	lis2dh12_get_axis(data, 5, &sample->z_raw, &sample->z_mg);

	return 0;
}
int main(void)
{
    i2c_scan(DEVICE_DT_GET(DT_BUS(DT_NODELABEL(myaccel))));

    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    /* OLED缓冲区，用于数字格式化输出 */
    char oled_buf[128];

    /* -------- UART配置 -------- */
    struct uart_config cfg = {
        .baudrate=115200,
        .parity=UART_CFG_PARITY_NONE,
        .stop_bits=UART_CFG_STOP_BITS_1,
        .data_bits=UART_CFG_DATA_BITS_8,
        .flow_ctrl=UART_CFG_FLOW_CTRL_NONE
    };

    if (!device_is_ready(uart_dev))
    {
        printk("uart_dev not ready!\n");
        while(1) k_sleep(K_FOREVER);
    }
    uart_configure(uart_dev, &cfg);
    uart_callback_set(uart_dev, uart_cd, NULL);
    uart_rx_enable(uart_dev,rx_buf,sizeof(rx_buf),100);

    /* -------- OLED初始化：只执行1次，放到while外面 -------- */
    if (!device_is_ready(dev))
    {
        printk("display device not ready!\n");
        while(1) k_sleep(K_FOREVER);
    }
    if (cfb_framebuffer_init(dev) != 0)
    {
        printk("cfb_framebuffer_init failed!\n");
        return -1;
    }

    /* -------- LIS2DH12初始化 -------- */
    struct lis2dh12_sample sample = {0};
    int ret;
    ret = lis2dh12_init();
    if (ret < 0) {
        printk("LIS2DH12: initialization failed: %d\n", ret);
        while (1) {
            k_sleep(K_FOREVER);
        }
    }

    while (1)
    {
        ret = lis2dh12_read(&sample);
        if (ret == 0)
        {
            //UART打印
            int len = snprintf(tx_buf, sizeof(tx_buf),
                "raw X=%d Y=%d Z=%d | mg X=%d Y=%d Z=%d\n",
                sample.x_raw, sample.y_raw, sample.z_raw,
                sample.x_mg, sample.y_mg, sample.z_mg);
            int tx_ret = uart_tx(uart_dev, tx_buf, len, SYS_FOREVER_MS);
            (void)tx_ret; //消除未使用警告

            //RTT打印
            printk("raw X=%d Y=%d Z=%d | mg X=%d Y=%d Z=%d\n",
                   sample.x_raw, sample.y_raw, sample.z_raw,
                   sample.x_mg, sample.y_mg, sample.z_mg);

            /* ========== OLED刷新输出 ========== */
            cfb_framebuffer_clear(dev, false);

            snprintf(oled_buf, sizeof(oled_buf),"R:%d %d %d",
                     sample.x_raw, sample.y_raw, sample.z_raw);
            cfb_print(dev, oled_buf, 0, 32);
			snprintf(oled_buf, sizeof(oled_buf),"M:%d %d %d",
                     sample.x_mg,sample.y_mg,sample.z_mg);
            cfb_print(dev, oled_buf, 0, 1);

            cfb_framebuffer_finalize(dev);
        }
        k_sleep(K_MSEC(100));
    }
}
