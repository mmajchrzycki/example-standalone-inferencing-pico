#include <stdio.h>
#include <string.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include "ei_classifier_types.h"
#include "porting/ei_classifier_porting.h"
#include "core1_thread.h"
#include "fake_data.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

bool linked = false;
bool first = true;
int features_offset = 0;

#ifndef DO_NOT_OUTPUT_TO_UART
// RX interrupt handler
uint8_t command[32] = {0};
bool start_flag = false;
int receive_index = 0;
uint8_t previous_ch = 0;

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
  ei_impulse_result_t res = {nullptr};

  stdio_init_all();
  setup_uart();
  core1_run();

  while (true)
  {
    ei_printf("Edge Impulse standalone inferencing on multicore (Raspberry Pi Pico)\n");

    while (1)
    {
      if(queue_try_remove(&results_queue, &res)) {
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                  res.timing.dsp, res.timing.classification, res.timing.anomaly);

        // print the predictions
        ei_printf("[");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        {
          ei_printf("%.5f", res.classification[ix].value);
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
        printf("%.3f", res.anomaly);
#endif
        printf("]\n");
        ei_sleep(2000);
      }

      // add samples to queue, one sample per main loop iteration (like using real sensor)
      for(int i = 0; i < EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME; i++) {
        if(queue_try_add(&data_queue, &features[features_offset + i]) == false) {
          // ei_printf("Data queue full!\n");
          break;
        }
      }
      if((features_offset += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) >= features_size) {
        features_offset = 0;
      }
    }
  }
}