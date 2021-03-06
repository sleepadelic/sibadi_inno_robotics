// пример тюнера на синтетическом нагревательном процессе

#include "PIDtuner.h"
PIDtuner tuner;

void setup() {
  Serial.begin(9600);

  // как работает данный алгоритм: устанавливает выходной сигнал равный СИГНАЛ
  // ждёт стабилизации значения с датчика по порогу скорости изменения ТОЧНОСТЬ СТАБИЛИЗАЦИИ за ПЕРИОД СТАБИЛИЗАЦИИ
  // уменьшает СИГНАЛ на величину СТУПЕНЬКА
  // ждёт время ПРОДОЛЖИТЕЛЬНОСТЬ ИМПУЛЬСА
  // увеличивает СИГНАЛ на величину СТУПЕНЬКА
  // начинает раскачивать систему и анализировать её поведение
  
 // направление, сигнал, ступенька, период стабилизации, точность стабилизации, продолж. импульса, период итерации
  tuner.setParameters(NORMAL, 10, 5, 1000, 0.04, 1000, 50);

  // направление: NORMAL - увеличение выходного сигнала увеличивает сигнал с датчика (например обогреватель, мотор)
  // REVERSE - увеличение выходного сигнала уменьшает сигнал с датчика (например холодильник, тормоз)
}

void loop() {
  float signal = process(tuner.getOutput());
  tuner.setInput(signal);
  tuner.compute();

  // выводит в порт текстовые отладочные данные, включая коэффициенты
  tuner.debugText();

  // выводит в порт данные для построения графиков, без коэффициентов
  //tuner.debugPlot();
}