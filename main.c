#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "xi2c.h"
#include "ssd1306.h"

#include <math.h>

#define I2C_MASTER_SCL_IO           26
#define I2C_MASTER_SDA_IO           25
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68
#define MPU6050_ACCEL_XOUT           		0x3B
#define MPU6050_ACCEL_YOUT           		0x3D
#define MPU6050_ACCEL_ZOUT           		0x3F
#define MPU6050_TEMP_OUT           			0x41

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B
#define MPU6050_PWR_MGMT_1_VALUE 			0x00

#define MPU6050_ACCEL_CONFIG_REG_ADDR     	0x1C
#define MPU6050_ACCEL_CONFIG_8G_VALUE		0x10

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


void app_main(void)
{
	double accelx;
	double accely;
	double accelz;
	//char acc[] = "|Acc| =   .   ";
	int Acc_module = 0;
	double acc_serial;
	int16_t accelx_aux;
	int16_t accely_aux;
	int16_t accelz_aux;
	uint8_t data[2];
    data[0] = 0;
    data[1] = 0;
    ESP_ERROR_CHECK(i2c_master_init());
    SSD1306_Init();

    ESP_ERROR_CHECK(mpu6050_register_write_byte (MPU6050_PWR_MGMT_1_REG_ADDR,MPU6050_PWR_MGMT_1_VALUE));
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_ACCEL_CONFIG_REG_ADDR, MPU6050_ACCEL_CONFIG_8G_VALUE));


    for(;;){
    	ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_XOUT, data, 2));
    	accelx_aux = (((data[0] <<8)) | data[1]);
    	accelx = accelx_aux;
    	accelx = accelx/4096;
    	ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_YOUT, data, 2));
    	accely_aux = (((data[0] <<8)) | data[1]);
    	accely = accely_aux;
    	accely = accely/4096;
    	ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_ACCEL_ZOUT, data, 2));
    	accelz_aux = (((data[0] <<8)) | data[1]);
    	accelz = accelz_aux;
    	accelz = accelz/4096;
    	//Acc_module =sqrt(pow(accelx,2) + pow(accely,2) + pow(accelz,2));

    	//acc_serial=Acc_module;



		printf("Accelerazione x = %f \n " ,accelx);
		printf("Accelerazione y = %f \n " ,accely);
		printf("Accelerazione z = %f \n " ,accelz);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}
