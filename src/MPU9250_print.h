#include <Arduino.h>
#include <Point3D>

template <typename T>
void printPoint(g3rb3n::Point3D<T>& a, uint8_t p)
{
  Serial.print('(');
  Serial.print(a.x, p);
  Serial.print(',');
  Serial.print(a.y, p);
  Serial.print(',');
  Serial.print(a.z, p);
  Serial.print(')');
}
