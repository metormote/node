
Reference Firmware Implementation for Node
===========================

http://iop.io
-------------

Node Sensor Interface
--------------

<table>
  <tr> <th>Pin</th><th>Name</th><th>Description</th></tr>
  <tr><td>1</td><td>VLD0 3.3V OUT</td><td>3.3V output for sensors</td></tr>
  <tr><td>2</td><td>GND</td><td>Ground</td></tr>
  <tr><td>3</td><td>IO_ADC0</td><td>General IO, A/D converter</td></tr>
  <tr><td>4</td><td>IO_ADC1</td><td>General IO, A/D converter</td></tr>
  <tr><td>5</td><td>IO_DAC0</td><td>General IO, D/A converter</td></tr>
  <tr><td>6</td><td>IO_DAC1</td><td>General IO, D/A converter</td></tr>
  <tr><td>7</td><td>SPI_MISO</td><td>SPI data in</td></tr>
  <tr><td>8</td><td>SPI_MOSI</td><td>SPI data out</td></tr>
  <tr><td>9</td><td>SPI_SCK</td><td>SPI clock</td></tr>
  <tr><td>10</td><td>SPI_CS</td><td>SPI chip select</td></tr>
  <tr><td>11</td><td>TWI_SDA</td><td>TWI/I2C data</td></tr>
  <tr><td>12</td><td>TWI_SCL</td><td>TWI/I2C clock</td></tr>
  <tr><td>13</td><td>UART_TX</td><td>Serial port transmit</td></tr>
  <tr><td>14</td><td>UART_RX</td><td>Serial port receive</td></tr>
  <tr><td>15</td><td>POWER_IN</td><td>3-5 V power supply</td></tr>
  <tr><td>16</td><td>GND</td><td>Ground</td></tr>
  <tr><td>17</td><td>PDI_DATA</td><td>Programming and debug interface</td></tr>
  <tr><td>18</td><td>3.3V MCU</td><td>Programming and debug interface</td></tr>
  <tr><td>19</td><td>RESET/PDI_CLK</td><td>Programming and debug interface</td></tr>
  <tr><td>20</td><td>GND</td><td>Ground</td></tr>
</table>



MCU Pinout Node
----------

<table>
  <tr> <th>Port Pin</th><th>MCU Function</th><th>Name</th><th>Comment</th></tr>
  <tr><td>PORTA0</td><td>ADC</td><td>POWER_VBAT_SENSE</td><td>Measures the battery voltage</td></tr>
  <tr><td>PORTA1</td><td>ADC</td><td>POWER_VUSB_SENSE</td><td>Measures the USB power connector voltage</td></tr>
  <tr><td>PORTA2</td><td>ADC</td><td>SENSOR_ADC0</td><td></td></tr>
  <tr><td>PORTA3</td><td>ADC</td><td>SENSOR_ADC1</td><td></td></tr>
  <tr><td>PORTA4</td><td>ADC</td><td>SENSOR_TEMP</td><td>Internal temperature sensor</td></tr>
  <tr><td>PORTA5</td><td>ADC</td><td>POWER_GOOD</td><td></td></tr>
  <tr><td>PORTA6</td><td>ADC</td><td>POWER_DCDC_ENABLE</td><td></td></tr>
  <tr><td>PORTA7</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTB0</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTB1</td><td>DAC</td><td>SENSOR_DAC0</td><td></td></tr>
  <tr><td>PORTB2</td><td>DAC</td><td>SENSOR_DAC1</td><td></td></tr>
  <tr><td>PORTB3</td><td>GPIO</td><td>UI_SWITCH</td><td>Board switch</td></tr>
  <tr><td>PORTB4</td><td>GPIO</td><td>UI_LED_GREEN</td><td></td></tr>
  <tr><td>PORTB5</td><td>GPIO</td><td>UI_LED_RED</td><td></td></tr>
  <tr><td>PORTB6</td><td>GPIO</td><td>UI_LED_YELLOW</td><td></td></tr>
  <tr><td>PORTB7</td><td>GPIO</td><td>UI_LED_BLUE</td><td></td></tr>
  <tr><td>PORTC0</td><td>SDA</td><td>SENSOR_SDA</td><td>Sensor interface TWI/I2C data</td></tr>
  <tr><td>PORTC1</td><td>SCL</td><td>SENSOR_SCL</td><td>Sensor interface TWI/I2C clock</td></tr>
  <tr><td>PORTC2</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTC3</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTC4</td><td></td><td></td><td>Not used (Dataflash Reset)</td></tr>
  <tr><td>PORTC5</td><td>MOSI</td><td>BLE_MOSI</td><td></td></tr>
  <tr><td>PORTC6</td><td>MISO</td><td>BLE_MISO</td><td></td></tr>
  <tr><td>PORTC7</td><td>SCK</td><td>BLE_SCK</td><td></td></tr>
  <tr><td>PORTD0</td><td>GPIO</td><td>BLE_ACTIVE</td><td></td></tr>
  <tr><td>PORTD1</td><td>GPIO</td><td>BLE_RESET</td><td></td></tr>
  <tr><td>PORTD2</td><td>RXD</td><td>ANT_UART_TX</td><td></td></tr>
  <tr><td>PORTD3</td><td>TXD</td><td>ANT_UART_RX</td><td></td></tr>
  <tr><td>PORTD4</td><td></td><td></td><td>Not used (Dataflash CS)</td></tr>
  <tr><td>PORTD5</td><td></td><td></td><td>Not used (Dataflash MOSI)</td></tr>
  <tr><td>PORTD6</td><td></td><td></td><td>Not used (Dataflash MISO)</td></tr>
  <tr><td>PORTD7</td><td></td><td></td><td>Not used (Dataflash SCK)</td></tr>
  <tr><td>PORTE0</td><td>INPUT</td><td>BLE_RDYN</td><td>Data ready</td></tr>
  <tr><td>PORTE1</td><td>OUTPUT</td><td>BLE_REQN</td><td>Request to send</td></tr>
  <tr><td>PORTE2</td><td>RXD</td><td>BLE_TXD</td><td>BLE Test Interface</td></tr>
  <tr><td>PORTE3</td><td>TXD</td><td>BLE_RXD</td><td>BLE Test Interface</td></tr>
  <tr><td>PORTE4</td><td>OUTPUT</td><td>SENSOR_CS</td><td>SPI Chip Select</td></tr>
  <tr><td>PORTE5</td><td>MOSI</td><td>SENSOR_MOSI</td><td></td></tr>
  <tr><td>PORTE6</td><td>MISO</td><td>SENSOR_MISO</td><td></td></tr>
  <tr><td>PORTE7</td><td>SCK</td><td>SENSOR_SCK</td><td></td></tr>
  <tr><td>PORTF0</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTF1</td><td></td><td></td><td>Not used</td></tr>
  <tr><td>PORTF2</td><td>RXD</td><td>SENSOR_RX</td><td></td></tr>
  <tr><td>PORTF3</td><td>TXD</td><td>SENSOR_TX</td><td></td></tr>
  <tr><td>PORTF4</td><td>OUTPUT</td><td>ANT_SUSPEND</td><td></td></tr>
  <tr><td>PORTF5</td><td>OUTPUT</td><td>ANT_SLEEP</td><td></td></tr>
  <tr><td>PORTF6</td><td>OUTPUT</td><td>ANT_RTS</td><td></td></tr>
  <tr><td>PORTF7</td><td>OUTPUT</td><td>ANT_RESET</td><td></td></tr>
</table>