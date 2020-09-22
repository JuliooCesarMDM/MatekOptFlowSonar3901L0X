#define MSP2_SENSOR_RANGEFINDER 0x1F01
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02

#define RANGEFINDER_OUT_OF_RANGE        (-1)
#define RANGEFINDER_NO_NEW_DATA         (-3)

typedef struct sbuf_s {
  uint8_t *ptr;
  uint8_t *end;
} sbuf_t;

typedef struct __attribute__((packed)) {
  uint8_t quality;
  int32_t distanceMm;
} mspRangefinderSensor_t;

typedef struct __attribute__((packed)) {
  uint8_t quality;
  int32_t motionX;
  int32_t motionY;
} mspOpflowSensor_t;

typedef struct opflowData_s {
  uint32_t deltaTime;
  int32_t  flowRateRaw[3];
  int16_t  quality;
} opflowData_t;

boolean RangeFinderhasNewData = false;
boolean OptFlowhasNewData = false;

static uint8_t dataSize;
static uint16_t cmdMSP;
static uint8_t serialBuffer[128];
uint8_t receiverIndex = 0;

int16_t Pointer = 0;


static uint32_t updatedTimeUs = 0;
static int32_t RangeFindersensorData = RANGEFINDER_NO_NEW_DATA;
static opflowData_t sensorData = {0};

void setup() {
  Serial.begin(115200);
}

void loop() {
  serialMSPreceive();
  Serial.print(mspRangefinderGetDistance());
  Serial.print(" ");
  Serial.print(sensorData.flowRateRaw[0]);
  Serial.print(" ");
  Serial.print(sensorData.flowRateRaw[1]);
  Serial.print(" ");
  Serial.println(sensorData.quality);
}

uint8_t Read8Bits() {
  return serialBuffer[Pointer++] & 0xff;
}

void serialMSPreceive() {
  uint8_t c;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  }
  c_state = IDLE;
  while (Serial.available()) {
    c = Serial.read();
    if (c_state == IDLE) {
      c_state = (c == '$') ? HEADER_START : IDLE;
    } else if   (c_state == HEADER_START) {
      c_state = (c == 'M') ? HEADER_M : IDLE;
    } else if   (c_state == HEADER_M) {
      c_state = (c == '<') ? HEADER_ARROW : IDLE;
    } else if (c_state == HEADER_ARROW) {
      if (c > 128) {
        c_state = IDLE;
      } else {
        dataSize = c;
        c_state = HEADER_SIZE;
      }
    } else if (c_state == HEADER_SIZE) {
      c_state = HEADER_CMD;
      cmdMSP = c;
    } else if (c_state == HEADER_CMD) {
      serialBuffer[receiverIndex++] = c;
      if (receiverIndex >= dataSize) {
        receiverIndex = 0;
        serialMSPCheck();
        c_state = IDLE;
      }
    }
  }
}

void serialMSPCheck() {
  Pointer = 0;

  switch (cmdMSP) {

    case MSP2_SENSOR_RANGEFINDER:
      mspProcessSensorCommand(cmdMSP, Read8Bits());
      break;

    case MSP2_SENSOR_OPTIC_FLOW:
      mspProcessSensorCommand(cmdMSP, Read8Bits());
      break;

  }
}

#define UNUSED(x) (void)(x)

uint8_t *sbufPtr(sbuf_t *buf) {
  return buf->ptr;
}

static void mspProcessSensorCommand(uint16_t cmdMSP, sbuf_t *src) {
  UNUSED(src);
  switch (cmdMSP) {
    case MSP2_SENSOR_RANGEFINDER:
      mspRangefinderReceiveNewData(sbufPtr(src));
      break;

    case MSP2_SENSOR_OPTIC_FLOW:
      mspOpflowReceiveNewData(sbufPtr(src));
      break;
  }
}

static int32_t mspRangefinderGetDistance(void) {
  if (RangeFinderhasNewData) {
    RangeFinderhasNewData = false;
    return (RangeFindersensorData > 0) ? RangeFindersensorData : RANGEFINDER_OUT_OF_RANGE;
  } else {
    return RANGEFINDER_NO_NEW_DATA;
  }
}

void mspRangefinderReceiveNewData(uint8_t *bufferPtr) {
  const mspRangefinderSensor_t *pkt = (const mspRangefinderSensor_t *)bufferPtr;
  RangeFindersensorData = pkt->distanceMm / 10;
  RangeFinderhasNewData = true;
}

void mspOpflowReceiveNewData(uint8_t *bufferPtr) {
  const uint32_t currentTimeUs = micros();
  const mspOpflowSensor_t *pkt = (const mspOpflowSensor_t*)bufferPtr;
  sensorData.deltaTime = currentTimeUs - updatedTimeUs;
  sensorData.flowRateRaw[0] = pkt->motionX;
  sensorData.flowRateRaw[1] = pkt->motionY;
  sensorData.flowRateRaw[2] = 0;
  sensorData.quality = (int)pkt->quality * 100 / 255;
  OptFlowhasNewData = true;
  updatedTimeUs = currentTimeUs;
}
