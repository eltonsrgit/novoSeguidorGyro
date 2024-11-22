#ifndef Bluetooth_H
#define Bluetooth_H


void ComandoBluetooth(String input) {
  if (input.indexOf("run") != -1) {
    running = true;
    BT.println("Seguidor em movimento.");
  }

  if (input.indexOf("stop") != -1) {
    running = false;
    seguidorStop();
    BT.println("Seguidor parou.");
  }

  if (input.indexOf("kp.change(") != -1 && input.indexOf(")") != -1) {
    int startIndex = input.indexOf("(") + 1;
    int endIndex = input.indexOf(")");
    float newValue = input.substring(startIndex, endIndex).toFloat();
    Kp = newValue;
    BT.println("Kp alterado para: " + String(Kp));
  }

  if (input.indexOf("ki.change(") != -1 && input.indexOf(")") != -1) {
    int startIndex = input.indexOf("(") + 1;
    int endIndex = input.indexOf(")");
    float newValue = input.substring(startIndex, endIndex).toFloat();
    Ki = newValue;
    BT.println("Ki alterado para: " + String(Ki));
  }

  if (input.indexOf("kd.change(") != -1 && input.indexOf(")") != -1) {
    int startIndex = input.indexOf("(") + 1;
    int endIndex = input.indexOf(")");
    float newValue = input.substring(startIndex, endIndex).toFloat();
    Kd = newValue;
    BT.println("Kd alterado para: " + String(Kd));
  }

  if (input.indexOf("v.change(") != -1 && input.indexOf(")") != -1) {
    int startIndex = input.indexOf("(") + 1;
    int endIndex = input.indexOf(")");
    vel_base = input.substring(startIndex, endIndex).toInt();
    BT.println("Velocidade base alterada para: " + String(vel_base));
  }
}

#endif