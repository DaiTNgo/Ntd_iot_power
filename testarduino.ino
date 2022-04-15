#include <DHT.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int DHTPIN = 2;      //Đọc dữ liệu từ DHT11 ở chân 2 trên mạch Arduino
const int DHTTYPE = DHT11; // Khai báo loại cảm biến, có 2 loại là DHT11 và DHT22

DHT dht(DHTPIN, DHTTYPE);

byte degree[8] = {
  0B01110,
  0B01010,
  0B01110,
  0B00000,
  0B00000,
  0B00000,
  0B00000,
  0B00000
};
#define ledRed 8
#define ledGreen 7
#define gasSensor A0
#define buzzer 4

int ad_value;     // bien luu gia tri adc
int button_value; // bien luu trang thai button
int per;
int sensorGasThreshold = 300;

QueueHandle_t queueGas;
QueueHandle_t queueDht11_Humidity;
QueueHandle_t queueDht11_Temperature;

void setup()
{
  Serial.begin(9600);

  dht.begin(); // Khởi động cảm biến

  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Nhiet do: ");
  lcd.setCursor(0, 1);
  lcd.print("Do am: ");
  lcd.createChar(1, degree);

  pinMode(gasSensor, INPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(buzzer, OUTPUT);

  queueGas = xQueueCreate(1, sizeof(int));

  queueDht11_Humidity = xQueueCreate(1, sizeof(float));
  queueDht11_Temperature = xQueueCreate(1, sizeof(float));

  dht.begin();

  xTaskCreate(dht11_task, "DHT11 task", 2048, NULL, 6, NULL);
  xTaskCreate(lcd_task, "lcd task", 2048, NULL, 5, NULL);
  xTaskCreate(read_gas_sensor_task, "Read gas", 2048, NULL, 6, NULL);
//  xTaskCreate(handle_gas_task, "Handle gas", 2048, NULL, 7, NULL);
  vTaskStartScheduler();
  for (;;) {}
}

void dht11_task(void *pvParameter)
{
  while (1)
  {
    float humidity = dht.readHumidity();       //Đọc độ ẩm
    float temperature = dht.readTemperature(); //Đọc nhiệt độ

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("dht11 read fail\n");
    }
    else
    {
      xQueueSend(queueDht11_Humidity, (void *)&humidity, (TickType_t)0);
      xQueueSend(queueDht11_Temperature, (void *)&temperature, (TickType_t)0);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void lcd_task(void *arg)
{
  float gBuffer_h;
  float gBuffer_t;
  while (1)
  {
    xQueueReceive(queueDht11_Humidity, &gBuffer_h, (TickType_t)0);
    xQueueReceive(queueDht11_Temperature, &gBuffer_t, (TickType_t)0);

    lcd.setCursor(10, 0);
    lcd.print(round(gBuffer_t));
    lcd.print(" ");
    lcd.print("C");
    lcd.setCursor(10, 1);
    lcd.print(round(gBuffer_h));
    lcd.print(" %");

//    Serial.print("Nhiet do: ");
//    Serial.println(gBuffer_t); // Xuất nhiệt độ
//    Serial.print("Do am: ");
//    Serial.println(gBuffer_h); // Xuất độ ẩm
//    Serial.println();          // Xuống hàng

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void read_gas_sensor_task(void *pvPara)
{
  while (1)
  {
    Serial.print("Read gas: ");
    int gas = analogRead(gasSensor);
    Serial.println(gas);
    xQueueSend(queueGas, (void *)&gas, (TickType_t)0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void handle_gas_task(void *pvPara)
{
  int gasBuffer;
  while (1)
  {
    Serial.println("Gas handle");
    xQueueReceive(queueGas, &gasBuffer, (TickType_t)0);

    if (gasBuffer < sensorGasThreshold)
    {
      Serial.println("Gas concentration in control");
      digitalWrite(ledRed, 0);
      digitalWrite(ledGreen, 1);
      digitalWrite(buzzer, 0);
    }
    else
    {
      Serial.println("Gas concentration too high!");
      digitalWrite(ledRed, 1);
      digitalWrite(ledGreen, 0);
      digitalWrite(buzzer, 1);

    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop()
{
}
