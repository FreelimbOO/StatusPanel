/********************************************/
/*****  Desktop PC Panel by FreelimbO  ******/
/*****  Date: 2021-08-25 Licence: MIT  ******/
/********************************************/

#include <SD.h>
#include <SPI.h>
#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_KBV.h> //Hardware-specific library

#define PIN_SD_CS 10 // Elegoo SD shields and modules: pin 10

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define DARK_BLUE 0x092A
#define	BLUE    0x001F
#define GREEN_BLUE 0x1736
#define	RED     0xF800
#define ORANGE  0xE3A2
#define	GREEN   0x07E0
#define DARK_GREEN 0x09A2
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define DARK_YELLOW 0x5241
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define CPU_TEMP 0
#define CPU_LOAD 1
#define GPU_TEMP 2
#define GPU_LOAD 3
#define MEM 4
#define GDDR 5

LCDWIKI_KBV my_lcd(ILI9486, A3, A2, A1, A0, A4); //model,cs,cd,wr,rd,reset

#define PIXEL_NUMBER  (my_lcd.Get_Display_Width()/4)
#define FILE_NUMBER 4
uint32_t bmp_offset = 0;
uint16_t s_width = my_lcd.Get_Display_Width();
uint16_t s_heigh = my_lcd.Get_Display_Height();

/*****  Digital photo frame mode.  *****/

uint16_t read_16(File fp) {
  uint8_t low;
  uint16_t high;
  low = fp.read();
  high = fp.read();
  return (high << 8) | low;
}

uint32_t read_32(File fp) {
  uint16_t low;
  uint32_t high;
  low = read_16(fp);
  high = read_16(fp);
  return (high << 16) | low;
}

bool analysis_bpm_header(File fp) {
  if (read_16(fp) != 0x4D42) {
    return false;
  }
  //get bpm size
  read_32(fp);
  //get creator information
  read_32(fp);
  //get offset information
  bmp_offset = read_32(fp);
  //get DIB infomation
  read_32(fp);
  //get width and heigh information
  uint32_t bpm_width = read_32(fp);
  uint32_t bpm_heigh = read_32(fp);
  if ((bpm_width != s_width) || (bpm_heigh != s_heigh)) {
    return false;
  }
  if (read_16(fp) != 1) {
    return false;
  }
  read_16(fp);
  if (read_32(fp) != 0) {
    return false;
  }
  return true;
}

void draw_bmp_picture(File fp) {
  uint16_t i, j, k, l, m = 0;
  uint8_t bpm_data[PIXEL_NUMBER * 3] = {0};
  uint16_t bpm_color[PIXEL_NUMBER];
  fp.seek(bmp_offset);
  for (i = 0; i < s_heigh; i++) {
    for (j = 0; j < s_width / PIXEL_NUMBER; j++) {
      m = 0;
      fp.read(bpm_data, PIXEL_NUMBER * 3);
      for (k = 0; k < PIXEL_NUMBER; k++) {
        bpm_color[k] = my_lcd.Color_To_565(bpm_data[m + 2], bpm_data[m + 1], bpm_data[m + 0]); //change to 565
        m += 3;
      }
      for (l = 0; l < PIXEL_NUMBER; l++) {
        my_lcd.Set_Draw_color(bpm_color[l]);
        my_lcd.Draw_Pixel(j * PIXEL_NUMBER + l, 479 - i);
      }
    }
  }
}

void setupSD() { //Init SD_Card
  pinMode(PIN_SD_CS, OUTPUT);
  if (!SD.begin(PIN_SD_CS)) {
    my_lcd.Set_Text_Back_colour(BLUE);
    my_lcd.Set_Text_colour(WHITE);
    my_lcd.Set_Text_Size(1);
    my_lcd.Print_String("SD Card Init fail!", 0, 0);
  }
}

/***** Graphic presentation of sensor readings. *****/

boolean newData = false;
float values[6];
float prevValues[6];
int nonzeroCounter = 0; // Count how many nonzeros are received.
uint16_t colorsCPU[5] = {MAGENTA, YELLOW, DARK_GREEN, RED, BLUE};
uint16_t colorsGPU[5] = {GREEN, GREEN_BLUE, DARK_GREEN,  BLUE, BLACK};

uint16_t rainbow[15] = {
  0x02FF, 0x04BF, 0x061F,
  0x07FF, 0x07F7, 0x07EF,
  0x5FE0, 0x97E0, 0xCFE0,
  0xFFE0, 0xFE60, 0xFCA0,
  0xFAE0, 0xF980, 0xF800
};

typedef struct st_c {
  int x;
  int y;
  int r;
  uint16_t c;
} cntr;
cntr prevC[2];




struct st_c drawCircules(int sensorType, int cx, int cy, int devi, uint16_t colors[5]) {
  int radius = 140;
  uint16_t color;

  for (int i = 0; i < 4; i++) {
    radius -= 2 * devi;
    color = colors[i];
    cy += devi;
    my_lcd.Set_Draw_color(color);
    my_lcd.Fill_Circle(cx, cy, radius);
  }
  my_lcd.Set_Text_Back_colour(colors[3]);
  my_lcd.Set_Text_colour(colors[4]);
  my_lcd.Set_Text_Size(5);

  switch (sensorType) {
    case CPU_TEMP:
    case CPU_LOAD:
      my_lcd.Print_String("CPU", cx - 37, cy - 67);
      break;
    case GPU_TEMP:
    case GPU_LOAD:
      my_lcd.Print_String("GPU", cx - 37, cy - 67);
      break;
  }

  my_lcd.Print_String("%", cx - 5 , cy + 40);
  cntr last_cntr = {.x = cx , .y = cy, .r = radius, .c = color};
  return last_cntr;
}

void drawTemp(int sensorType, struct st_c c, int devi) {
  cntr innerC = c;
  cntr outerC = {c.x, c.y - devi, c.r + 2 * devi, c.c};
  float x1, y1, x2, y2;
  float temp = values[sensorType];
  float maxDegree = temp / 100.0 * 360.0;
  maxDegree = min(360, (int)maxDegree);
  float prevMaxDegree = prevValues[sensorType] / 100.0 * 360.0;
  int stepping = 0;
  if (maxDegree != prevMaxDegree) { // Only plot the difference
    stepping = (maxDegree > prevMaxDegree) ? (1) : (-1);
    for (int i = prevMaxDegree; i != maxDegree; i += stepping) {
      float rad = ((float)i + 90.0) / 180.0 * PI;
      x1 = innerC.x + innerC.r * cos(rad); //Polar to cardinal
      y1 = innerC.y + innerC.r * sin(rad);
      x2 = outerC.x + outerC.r * cos(rad);
      y2 = outerC.y + outerC.r * sin(rad);
      if (stepping >= 0) {
        //tft.drawLine(x1, y1, x2, y2, tft.color565(i,i,254));
        my_lcd.Set_Draw_color(rainbow[(int)min(14, (float)i / 360.0 * 19.0)]);
        my_lcd.Draw_Line(x1, y1, x2, y2);
      } else {
        my_lcd.Set_Draw_color(DARK_GREEN);
        my_lcd.Draw_Line(x1, y1, x2, y2);
      }
    }
    prevValues[sensorType] = values[sensorType];
  }
  values[sensorType] = 0.0;
}

void updateNumber(int censorType, int x, int y, int16_t color) {
  my_lcd.Set_Text_colour(color);
  my_lcd.Set_Text_Size(6);

  if (censorType == CPU_LOAD) {
    my_lcd.Set_Text_Back_colour(colorsCPU[3]);
  } else {
    my_lcd.Set_Text_Back_colour(colorsGPU[3]);
  }
  my_lcd.Print_Number_Float(values[censorType], 0, x, y, '.', 0, ' ');
  values[censorType] = 0.0;
  newData = false;
}

float memHistory[25];
float gddrHistory[25];
void plotHist(int sensorType, int x, int y, uint16_t c) {
  int width = 50, height = 100;
  int lbx = x, lby = y;

  my_lcd.Set_Draw_color(BLACK);
  my_lcd.Fill_Rectangle(lbx, lby, lbx + width, lby + height + 2);

  float *ptr0;
  switch (sensorType) {
    case MEM:
      ptr0 = memHistory;
      break;
    case GDDR:
      ptr0 = gddrHistory;
      break;
  }

  for (byte i = 0; i < 25; i++) {
    int len0 = (int)ptr0[i];
    int top0 = lby + height - len0;
    my_lcd.Set_Draw_color(c);
    my_lcd.Draw_Fast_VLine(lbx + 2 * i, top0, len0);
    my_lcd.Draw_Fast_VLine(lbx + 2 * i + 1, top0, len0);
    if (i < 24) {
      my_lcd.Set_Draw_color(CYAN);
      my_lcd.Draw_Line(lbx + 2 * i, top0, lbx + 2 * (i + 1) + 1, lby + height - (int)ptr0[i + 1]);
      ptr0[i] = ptr0[i + 1];
    } else {
      ptr0[i] = values[sensorType];
      values[sensorType] = 0;
    }
  }
}

void setupStat() {
  my_lcd.Fill_Screen(BLACK);
  prevC[0] = drawCircules(CPU_TEMP, 100, 105, 6, colorsCPU);
  prevC[1] = drawCircules(GPU_TEMP, 190, 340, 6, colorsGPU);
}

void updateStat() {
  int halfw = 0.85 * prevC[0].r, halfh = 0.5 * prevC[0].r;

  drawTemp(CPU_TEMP, prevC[0], 6);
  updateNumber(CPU_LOAD, prevC[0].x - halfw + 12, prevC[0].y - 15, WHITE);
  plotHist(MEM, prevC[0].x + halfw + 16 + 40, prevC[0].y - 15 - 70, MAGENTA);

  drawTemp(GPU_TEMP, prevC[1], 6);
  updateNumber(GPU_LOAD, prevC[1].x - halfw + 12, prevC[1].y - 15, CYAN);
  plotHist(GDDR, prevC[1].x - halfw - 100, prevC[1].y - 40, BLUE);
}

/*****  Receiving serial port messages,  *****/

unsigned long prevTime, currTime;
void recvWithStartEndMarkers() {
  byte rid = 0; byte cid = 0;
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  char receivedChars[4];
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc == endMarker) {
        recvInProgress = false;
        values[rid] = atof(receivedChars);
        for (byte i = 0; i < 4; i++) receivedChars[i] = ' ';
        if (values[rid] != 0) nonzeroCounter++;
        rid = 0;
        cid = 0;
        newData = true;
      } else if (rc == ',') {
        values[rid] = atof(receivedChars);
        for (byte i = 0; i < 4; i++) receivedChars[i] = ' ';
        if (values[rid] != 0) nonzeroCounter++;
        rid++;
        cid = 0;
      } else {
        receivedChars[cid] = rc;
        cid++;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
      prevTime = millis();
    }
  }
}

/***** The two core functions of Arduino.  *****/

boolean enteredS3 = false;
void setup(void) {
  Serial.begin(115200);
  Serial.println("<Arduino is ready!>");

  my_lcd.Init_LCD();
  my_lcd.Fill_Screen(BLUE);
  my_lcd.Set_Rotation(0);

  setupSD();
  setupStat();
}

byte skipCounter = 0;
byte imgCounter = 0;
void loop(void) {
  Serial.flush();
  currTime = millis();
  if (currTime - prevTime > 6000) { // Test if the host PC enters S3/S4
    newData = false;
    prevValues[CPU_TEMP] = 0;
    prevValues[GPU_TEMP] = 0;
  }
  Serial.println("<Req...>");
  recvWithStartEndMarkers();
  byte threshold = 5;
  if (nonzeroCounter == 0) { // Boundry check of [0, threshold]
    skipCounter = (skipCounter < threshold) ? (skipCounter + 1) : (threshold);
  } else {
    skipCounter = (skipCounter > 0) ? (skipCounter - 1) : (0);
  }

  if (skipCounter < threshold) {
    if (enteredS3) {
      setupStat();
      enteredS3 = false;
    }
    updateStat();
  }
  else {
    if (!enteredS3) {
      setupSD();
      enteredS3 = true;
      imgCounter = 1;
    }

    File bmp_file = SD.open((String("0") + imgCounter + String(".bmp")).c_str());
    imgCounter = (imgCounter == FILE_NUMBER) ? (1) : (imgCounter + 1);
    if (!bmp_file) {
      my_lcd.Set_Text_Back_colour(BLUE);
      my_lcd.Set_Text_colour(WHITE);
      my_lcd.Set_Text_Size(1);
      my_lcd.Print_String("didnt find BMPimage!", 0, 10);
      while (1);
    }
    if (!analysis_bpm_header(bmp_file)) {
      my_lcd.Set_Text_Back_colour(BLUE);
      my_lcd.Set_Text_colour(WHITE);
      my_lcd.Set_Text_Size(1);
      my_lcd.Print_String("bad bmp picture!", 0, 0);
      return;
    }
    draw_bmp_picture(bmp_file);
    bmp_file.close();

    delay(2000);
  }

  nonzeroCounter = 0;
  delay(2000);
}
