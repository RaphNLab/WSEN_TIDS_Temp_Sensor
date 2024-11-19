#include "drivers/timer_drv.h"
#include "drivers/uart_drv.h"
#include "drivers/led_driver.h"
#include "WSEN_TIDS_TEMP_SENSOR.h"

int main(void)
{
	timer_sleep_setup();
	led_setup();

	wsenTdis_i2c1Setup();
	serial_debug_setup();


	wsenTdis_continuousMode(WSEN_TIDS_ODR_50_HZ);
	while (1)
	{
		float temperature = 0;

		if(wsenTdis_readDeviceId() == WSEN_TIDS_OK)
		{
			wsenTdis_readTemperature(&temperature);
			printf("%.2f°C\n", temperature);
		}
		sleep_ms(100);
	}
	return 0;
}
