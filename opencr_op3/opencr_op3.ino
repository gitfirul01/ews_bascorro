/*
 *  opencr_op3
 *
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */
#include "dxl_node_op3.h"


extern void dxl_hw_tx_enable(void);

void setup()
{
  Serial.begin(115200);
  Serial.println("op3 start");
  dxl_node_op3_init();
  
  // paksa power 12V enable
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
}

void loop()
{
  dxl_node_op3_loop();
//  Serial.println("hi");
//  Serial.print("x: "); Serial.println(orientation_r());
//  Serial.print("y: "); Serial.println(orientation_p());
//  Serial.print("z: "); Serial.println(orientation_y());

//   Serial.print("x: "); Serial.println(orientation_gyroX());
//   Serial.print("y: "); Serial.println(orientation_gyroY());
//   Serial.print("z: "); Serial.println(orientation_gyroZ());

//   Serial.print("x: "); Serial.println(orientation_accX());
//   Serial.print("y: "); Serial.println(orientation_accY());
//   Serial.print("z: "); Serial.println(orientation_accZ());


//  Serial.println();
//  delay(300);
}
