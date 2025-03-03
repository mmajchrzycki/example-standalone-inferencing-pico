#include "ei_run_classifier.h"

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>
#include <stdio.h>

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = 25;

bool linked = false;
bool first = true;
uint16_t send_index = 0;

#ifndef DO_NOT_OUTPUT_TO_UART
// RX interrupt handler
uint8_t command[32] = {0};
bool start_flag = false;
int receive_index = 0;
uint8_t previous_ch = 0;

static const float features[] = {
    // copy raw features here (for example from the 'Live classification' page)

};

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

void on_uart_rx()
{
  uint8_t current_ch = 0;
  while (uart_is_readable(UART_ID))
  {
    current_ch = uart_getc(UART_ID);
    //    printf("%c \n", current_ch);
    if (start_flag)
    {
      command[receive_index++] = current_ch;
    }
    if (current_ch == 0xf4 && previous_ch == 0xf5)
    {
      start_flag = true;
    }
    else if (current_ch == 0x0a && previous_ch == 0x0d)
    {
      start_flag = false;
      // add terminator
      command[receive_index - 2] = '\0';

      receive_index = 0;
      if (strcmp("IND=BLECONNECTED", (const char *)command) == 0)
      {
        linked = true;
      }
      else if (strcmp("IND=BLEDISCONNECTED", (const char *)command) == 0)
      {
        linked = false;
      }
      printf("%s\n", command);
    }
    previous_ch = current_ch;
  }
}

void setup_uart()
{
  // Set up our UART with the required speed.
  uint baud = uart_init(UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, false);
  // Set up a RX interrupt
  // We need to set up the handler first
  // Select correct interrupt for the UART we are using
  int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(UART_ID, true, false);
}
#else
void setup_uart()
{
}
#endif

int main()
{
  stdio_usb_init();
  setup_uart();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  ei_impulse_result_t result = {nullptr};

  while (true)
  {
    ei_printf("Edge Impulse standalone inferencing (Raspberry Pi Pico)\n");

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
    {
      ei_printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
                EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
      return 1;
    }

    while (1)
    {
      // blink LED
      gpio_put(LED_PIN, !gpio_get(LED_PIN));

      // the features are stored into flash, and we don't want to load everything into RAM
      signal_t features_signal;
      features_signal.total_length = sizeof(features) / sizeof(features[0]);
      features_signal.get_data = &raw_feature_get_data;

      // invoke the impulse
      EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, true);

      ei_printf("run_classifier returned: %d\n", res);

      if (res != 0)
        return 1;

      ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

      // print the predictions
      ei_printf("[");
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
      {
        ei_printf("%.5f", result.classification[ix].value);
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf(", ");
#else
        if (ix != EI_CLASSIFIER_LABEL_COUNT - 1)
        {
          ei_printf(", ");
        }
#endif
      }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
      printf("%.3f", result.anomaly);
#endif
      printf("]\n");

      ei_sleep(2000);
    }
  }
}