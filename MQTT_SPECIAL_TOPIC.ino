#include <SPI.h>
#include <mcp_can.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// WiFi Credentials
const char* ssid = "Galaxy A35 5G 20CC";
const char* password = "44444444";

// HiveMQ public broker
const char* mqtt_server = "broker.hivemq.com";

// CAN SPI CS pin
const int SPI_CS_PIN = D2;
MCP_CAN CAN(SPI_CS_PIN);  // MCP2515 interface

WiFiClient espClient;
PubSubClient client(espClient);

// Connect to Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Reconnect to MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("NodeMCU_CAN_Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("CAN BUS init OK!");
  } else {
    Serial.println("CAN BUS init FAIL!");
    while (1);
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Receiver Ready");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    CAN.readMsgBuf(&rxId, &len, rxBuf);

    //if (rxId == 0x0A9 && len == 8) {
      uint16_t ir_data  = (rxBuf[0] << 8) | rxBuf[1];
      uint16_t pir_data = (rxBuf[2] << 8) | rxBuf[3];
      uint16_t gas_data = (rxBuf[4] << 8) | rxBuf[5];

      Serial.print("IR: ");
      Serial.print(ir_data);
      Serial.print(" | PIR: ");
      Serial.print(pir_data);
      Serial.print(" | GAS: ");
      Serial.println(gas_data);

      // Publish to HiveMQ MQTT
      char msg[10];

      snprintf(msg, sizeof(msg), "%d", ir_data);
      client.publish("can/ir", msg);

      snprintf(msg, sizeof(msg), "%d", pir_data);
      client.publish("can/pir", msg);

      snprintf(msg, sizeof(msg), "%d", gas_data);
      client.publish("can/gas", msg);
    //}
  }
}
