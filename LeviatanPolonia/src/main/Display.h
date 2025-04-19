#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Display {
private:
  Adafruit_SSD1306 pantalla;

public:
  Display() : pantalla(128, 64, &Wire, -1) {} // Resolución 128x64

  void begin() {
    if (!pantalla.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("No se encontró el display OLED"));
      while (true);
    }
    pantalla.clearDisplay();
    pantalla.display();
  }

  void mostrarTexto(const String& texto, int x = 0, int y = 0, int tam = 1) {
    pantalla.clearDisplay();
    pantalla.setTextSize(tam);
    pantalla.setTextColor(SSD1306_WHITE);
    pantalla.setCursor(x, y);
    pantalla.println(texto);
    pantalla.display();
  }

  void mostrarEstado(const String& estado, int rpmIzq, int rpmDer) {
    pantalla.clearDisplay();
    pantalla.setTextSize(1);
    pantalla.setTextColor(SSD1306_WHITE);
    pantalla.setCursor(0, 0);
    pantalla.println("Estado: " + estado);
    pantalla.println("RPM Izq: " + String(rpmIzq));
    pantalla.println("RPM Der: " + String(rpmDer));
    pantalla.display();
  }

  void limpiar() {
    pantalla.clearDisplay();
    pantalla.display();
  }

void mostrarMapa() {
  pantalla.clearDisplay();
  pantalla.setTextSize(0.5);
  pantalla.setTextColor(SSD1306_WHITE);

  const char* filas[] = {
    "0 0 0 0 0",
    "0 0 0 0 0",
    "0 0 M 0 0",
    "0 0 0 0 0",
    "0 0 0 0 0"
  };

  for (int i = 0; i < 5; i++) {
    pantalla.setCursor(0, i * 16);  // 6 píxeles por línea con textSize 1
    pantalla.println(filas[i]);
  }

  pantalla.display();
}

};

#endif
