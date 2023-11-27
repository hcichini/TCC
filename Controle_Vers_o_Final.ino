#include <arduinoFFT.h>

#define SAMPLES 1024
#define SAMPLING_FREQ 1024 
#define NUM_READINGS 1    
#define NUM_PEAKS 10        
#define SEU_LIMITE 1.2     
#define Val 27
#define Mot 26
#define Mot2 25

arduinoFFT FFT = arduinoFFT(); 

unsigned int sampling_period_us;
unsigned long micros_previous;
double vReal[SAMPLES];
double vImag[SAMPLES];
int option = 0;
double soma_frequencias = 0.0;
double leituras[NUM_READINGS][SAMPLES / 2];
double media_frequencias = 0;
bool CHECK = false;

const int freqTable[SAMPLES / 2] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
  41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78,
  79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
  114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144,
  145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
  176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206,
  207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237,
  238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268,
  269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299,
  300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330,
  331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360, 361,
  362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392,
  393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423,
  424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454,
  455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485,
  486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503, 504, 506, 507, 508, 509, 510, 511, 512};

void setup() {
  Serial.begin(115200);
  pinMode(Val, OUTPUT);
  pinMode(Mot, OUTPUT);
  pinMode(Mot2, OUTPUT);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

  micros_previous = micros();
}

void loop() {
  unsigned long micros_current = micros();
  float tensao = analogRead(36)*(3.3/4095.0);
  Serial.println(tensao);
  if (tensao > SEU_LIMITE) {
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = analogRead(36);
      vImag[i] = 0;
      while (micros() - micros_current < sampling_period_us) {
      }
      micros_current += sampling_period_us;
    }
    
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);

    for (int i = 0; i < (SAMPLES / 2); i++) {
      double magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
      leituras[0][i] = magnitude;

      for (int j = NUM_READINGS - 1; j > 0; j--) {
        leituras[j][i] = leituras[j - 1][i];
      }
    }
    double picos[NUM_READINGS][NUM_PEAKS];
    int indices_picos[NUM_READINGS][NUM_PEAKS];
    for (int k = 0; k < NUM_READINGS; k++) {
      for (int i = 0; i < NUM_PEAKS; i++) {
        picos[k][i] = 0.0;
        indices_picos[k][i] = 0;
      }
    }
    for (int k = 0; k < NUM_READINGS; k++) {
      for (int i = 0; i < (SAMPLES / 2); i++) {
        if (leituras[k][i] > picos[k][NUM_PEAKS - 1] && i != 0) {
          picos[k][NUM_PEAKS - 1] = leituras[k][i];
          indices_picos[k][NUM_PEAKS - 1] = i;

          for (int j = NUM_PEAKS - 1; j > 0; j--) {
            if (picos[k][j] > picos[k][j - 1]) {
              double temp_pico = picos[k][j];
              int temp_indice = indices_picos[k][j];
              picos[k][j] = picos[k][j - 1];
              indices_picos[k][j] = indices_picos[k][j - 1];
              picos[k][j - 1] = temp_pico;
              indices_picos[k][j - 1] = temp_indice;
            }
          }
        }
      }
    }
    double soma_frequencias = 0.0;
    Serial.println("Picos identificados:");
    for (int k = 0; k < NUM_READINGS; k++) {
      Serial.print("Leitura ");
      Serial.println(k);
      for (int i = 0; i < NUM_PEAKS; i++) {
        double frequencia_Hz = freqTable[indices_picos[k][i]];
        if (frequencia_Hz != 0) {
          soma_frequencias += frequencia_Hz;
          Serial.print("Pico ");
          Serial.print(i);
          Serial.print(": Frequência=");
          Serial.print(frequencia_Hz);
          Serial.print(" Hz, Magnitude=");
          Serial.println(picos[k][i]);
        }
      }
    }
    media_frequencias = soma_frequencias / NUM_PEAKS;

  }
    Serial.print("Média das frequências: ");
    Serial.println(media_frequencias);

    if (media_frequencias >= 0 && media_frequencias <= 1) {
      option = 0;
    } else if (media_frequencias > 1 && media_frequencias <= 70) {
      option = 1;
    } else if (media_frequencias > 70 && media_frequencias <= 510) {
      option = 2;
    }
    if(CHECK == false){
      switch (option) {
        case 0:
          Serial.println("Mão Parada");
          break;
  
        case 1:
          Serial.println("Mão Fechou");
          digitalWrite(Mot, HIGH);
          digitalWrite(Mot2, HIGH);
          delay(2000);
          digitalWrite(Mot, LOW);
          digitalWrite(Mot2, LOW);
          delay(50);
          CHECK = true;
          break;
  
        case 2:
          Serial.println("Não Mexer");
          break;
                    
        default:
          Serial.println("Opção inválida.");
          break;
      }
    media_frequencias = 0;
      }
      else{
       switch (option) {
        case 0:
          Serial.println("Mão Parada");  
          break;
  
        case 1:
          Serial.println("Mão Abriu");
          digitalWrite(Val, HIGH);
          delay(500);
          digitalWrite(Val, LOW);
          delay(5);
          CHECK = false;
          break;
  
        case 2:
          Serial.println("Não Mexer");
          break;
          
        default:
          Serial.println("Opção inválida.");
          break;
      }
      media_frequencias = 0;
        }
  delay(100);
}
