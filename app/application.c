#include <application.h>
#include <math.h>

#include "angle_average.h"
/*
    Odesílání každých 5 minut , 300 sekund , 300 000 ms
    Vzorkování směru větru a rychlosti: 5 sekund

    300s / 5s = 60 vzorků
    60s / 5s = 12 vzorků

    vzorkování 15 sekund
    (5min)300s / 15s = 20 vzorků

*/


typedef enum
{
    SIGFOX_HEADER_RESET = 0,
    SIGFOX_HEADER_MOTION = 1

} sigfox_header_t;

bc_led_t led;
bc_module_sigfox_t sigfox_module;

float windSpeedAverage = 0;
float windSpeedMaximum = 0;
float windAngleAverage = 0;
float batteryVoltage = 0;

#define MAIN_TASK_PERIOD_SECONDS 15
#define SIGFOX_TRANSMIT_PERIOD_MINUTES 5
#define WIND_DATA_STREAM_SAMPLES 20

// Data stream for wind speed averaging
BC_DATA_STREAM_FLOAT_BUFFER(stream_buffer_wind_speed, WIND_DATA_STREAM_SAMPLES)
bc_data_stream_t stream_wind_speed;

// Wind voltage table
// This was measured with original resistor in wind direction sensor and internal MCU pullup
// Some values are too close to each other. Better solution will be to use external 4k7 hardware pullup
// and edit this table to the real ADC values.
float windVoltageTable[16] = {
    1.463, // 0° North
    0.469, // 22.5°
    0.565, // 45°
    0.078, // 67.5°
    0.087, // 90° East
    0.062, // 112.5°
    0.179, // 135°
    0.118, // 157.5°
    0.298, // 180° South
    0.247, // 202.5°
    0.935, // 225°
    0.859, // 247.5°
    2.372, // 270° West
    1.650, // 292.5°
    1.970, // 315°
    1.152  // 337.5°
};

// Last angle and speed for debug
float windAngle;
float windSpeed;

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *param);

void sigfox_transmit_task(void *param)
{
    (void) param;

    if (!bc_module_sigfox_is_ready(&sigfox_module))
    {
        bc_scheduler_plan_current_now();
        return;
    }

    //bc_led_set_mode(&led, BC_LED_MODE_ON);
    bc_led_pulse(&led, 500);

    uint16_t speed = (uint16_t)(windSpeedAverage * 10.0f);
    uint16_t speedMaximum = (uint16_t)(windSpeedMaximum * 10.0f);
    uint16_t angle = (uint16_t)windAngleAverage;
    uint16_t battery = (uint16_t)(batteryVoltage * 1000);

    uint8_t i = 0;
    uint8_t buffer[8];

    buffer[i++] = speed & 0xFF;
    buffer[i++] = (speed >> 8) & 0xFF;
    buffer[i++] = speedMaximum & 0xFF;
    buffer[i++] = (speedMaximum >> 8) & 0xFF;
    buffer[i++] = angle & 0xFF;
    buffer[i++] = (angle >> 8) & 0xFF;
    buffer[i++] = battery & 0xFF;
    buffer[i++] = (battery >> 8) & 0xFF;

    bc_module_sigfox_send_rf_frame(&sigfox_module, buffer, sizeof(buffer));
    bc_scheduler_plan_current_relative(1000 * 60 * SIGFOX_TRANSMIT_PERIOD_MINUTES);

    // Reset the maximum wind speed
    windSpeedMaximum = 0;
}



void sigfox_module_event_handler(bc_module_sigfox_t *self, bc_module_sigfox_event_t event, void *event_param)
{
    (void) self;
    (void) event_param;

    if (event == BC_MODULE_SIGFOX_EVENT_ERROR)
    {
        bc_led_pulse(&led, 0);
        bc_led_set_mode(&led, BC_LED_MODE_BLINK);
    }
    else if (event == BC_MODULE_SIGFOX_EVENT_SEND_RF_FRAME_DONE)
    {
        bc_led_set_mode(&led, BC_LED_MODE_OFF);
    }
}


float windVoltageToAngle(float voltage)
{
    float smallestDifferenceValue = 360.0f; // Set big number
    float smallestDifferenceAngle;

    uint32_t i;

    for(i = 0; i < 16; i++)
    {
        float currentDifference = fabs(voltage - windVoltageTable[i]);
        if(smallestDifferenceValue > currentDifference)
        {
            smallestDifferenceValue = currentDifference;
            smallestDifferenceAngle = 22.50f * i;
        }
    }

    return smallestDifferenceAngle;
}

float windAdc = 0;
float angleAverageOut;

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *param)
{
    (void)param;
    (void)channel;

     if (event == BC_ADC_EVENT_DONE)
     {
         if(channel == BC_ADC_CHANNEL_A5)
         {
            // Disable pullup
            bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_NONE);

            bc_adc_get_result(BC_ADC_CHANNEL_A5, &windAdc);
            windAngle = windVoltageToAngle(windAdc);

            angle_average_add(windAngle);
            windAngleAverage = angle_average_get();

            // Start battery measurement, default library collides sometimes with ADC measurement
            // so I use manual solution
            //_bc_module_battery_measurement(1);
            bc_gpio_set_output(BC_GPIO_P1, 1);
            bc_adc_async_read(BC_ADC_CHANNEL_A0);
         }
         if(channel == BC_ADC_CHANNEL_A0)
         {
             #define ADC_VALUE_TO_VOLTAGE(__RESULT__)   (((__RESULT__) * (1 / 0.13)))
            float val;
            bc_adc_get_result(BC_ADC_CHANNEL_A0, &val);
            //_bc_module_battery_measurement(0);
            bc_gpio_set_output(BC_GPIO_P1, 0);

            batteryVoltage = ADC_VALUE_TO_VOLTAGE(val);
         }

     }
}

void application_init(void)
{
    //bc_data_stream_init(&stream_wind_direction, 1/*WIND_DATA_STREAM_SAMPLES*/, &stream_buffer_wind_direction);
    bc_data_stream_init(&stream_wind_speed, 1/*WIND_DATA_STREAM_SAMPLES*/, &stream_buffer_wind_speed);

    bc_led_init(&led, BC_GPIO_LED, false, false);

    // Pulse counter
    bc_pulse_counter_init(BC_MODULE_SENSOR_CHANNEL_A, BC_PULSE_COUNTER_EDGE_FALL);
	bc_pulse_counter_set_event_handler(BC_MODULE_SENSOR_CHANNEL_A, NULL, NULL);

    bc_module_sensor_set_mode(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_MODE_INPUT);
    // Pullup is enabled in the task just before measuring
    //bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_UP_INTERNAL);

    // Initialize ADC channel - wind direction
    bc_adc_init(BC_ADC_CHANNEL_A5, BC_ADC_FORMAT_FLOAT);
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A5, adc_event_handler, NULL);

    // Battery voltage
    bc_gpio_init(BC_GPIO_P1);
    //_bc_module_battery_measurement(0);
    bc_gpio_set_output(BC_GPIO_P1, 0);
    bc_gpio_set_mode(BC_GPIO_P1, BC_GPIO_MODE_OUTPUT);

    bc_adc_init(BC_ADC_CHANNEL_A0, BC_ADC_FORMAT_FLOAT);
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A0, adc_event_handler, NULL);

    // Sigfox
    bc_module_sigfox_init(&sigfox_module, BC_MODULE_SIGFOX_REVISION_R2);
    bc_module_sigfox_set_event_handler(&sigfox_module, sigfox_module_event_handler, NULL);
    bc_scheduler_register(sigfox_transmit_task, NULL, 0);

    // Do a single blink to signalize that module is working
    bc_led_pulse(&led, 1000);
}



void application_task()
{
    // Enable pullup during wind direction measurement
    bc_module_sensor_set_pull(BC_MODULE_SENSOR_CHANNEL_B, BC_MODULE_SENSOR_PULL_UP_INTERNAL);
    bc_adc_async_read(BC_ADC_CHANNEL_A5);

    float counter = (float)bc_pulse_counter_get(BC_MODULE_SENSOR_CHANNEL_A);
    bc_pulse_counter_reset(BC_MODULE_SENSOR_CHANNEL_A);

    // Get current wind speed, one pulse per second ~ 2.4kmph
    windSpeed = (counter / ((float)MAIN_TASK_PERIOD_SECONDS)) * 0.66666f; // 2.4km/h ~ 0.66666m/s

    // Save maximum wind speed value
    if(windSpeed > windSpeedMaximum)
    {
        windSpeedMaximum = windSpeed;
    }

    // Add value to stream and get average
    bc_data_stream_feed(&stream_wind_speed, &windSpeed);
    bc_data_stream_get_average(&stream_wind_speed, &windSpeedAverage);

    bc_scheduler_plan_current_relative(MAIN_TASK_PERIOD_SECONDS * 1000);
}
