#!/usr/bin/env python3

import serial
from serial.tools import list_ports
import subprocess

Config = {
  "port": "/dev/ttyUSB0",
  "baudrate": 9600,
  "timeout": 1
}

__SERIAL_NAME__ = "UPS power unit"

# UPS電源設定項目
## 電源からPCへの出力
CTS_OUTPUT_AC_FINE = True
CTS_OUTPUT_AC_DOWN = False
DCD_OUTPUT_BATTERY_FINE = True
DCD_OUTPUT_BATTERY_LOW = False
## PCから電源への入力
TTL_INPUT_DC_FINE = "HIGH"
TTL_INPUT_DC_DOWN = "LOW"

""" UPS電源通信管理クラス
"""
class Ups_serial:
  def __init__(self):
    self.config = Config
    self.is_alive = False
    
  """ シリアル接続
  """
  def connect(self):
    print("[LOG] Connecting to " + __SERIAL_NAME__ + "...")
    try:
      self.serial = serial.Serial(
        port=self.config.port,
        baudrate=self.config.baudrate,
        timeout=self.config.timeout
      )
      self.serial.open()
      self.is_alive = True
      print("[LOG] Success to connect to " + __SERIAL_NAME__)
    except:
      self.is_alive = False
      print("[LOG] Failed to connect to " + __SERIAL_NAME__)
      
  """ シリアル接続解除
  """
  def disconnect(self):
    self.is_alive = False
    self.serial.close()
    print("[LOG] Disconnected to " + __SERIAL_NAME__)
  
  # FIXME: ドメイン的にはここに存在していいものではない
  """ シャットダウン処理実行
  """
  def shutdown(self):
    try:
      print("[LOG] Shutdowning...")
      subprocess.call(["shutdown", "-t", "1"]) # Dockerコンテナ内からこれの動作確認はできない？
    except:
      return False
    
  """ バッテリー駆動状態かどうか取得
  """
  def is_battery_mode(self):
    if self.is_alive == True: # TODO: 生きてないときにエラー処理
      is_powered = self.serial.cts # FIXME: シリアル通信が来なかった時にTimeout文だけ待ってしまう？受信できない場合にEOFの例外処理が必要か？
      if is_powered == CTS_OUTPUT_AC_DOWN:
        return True
      if is_powered == CTS_OUTPUT_AC_FINE:
        return False
      
  """ バッテリー充電状態を取得
  """
  def is_battery_fine(self):
    if self.is_alive == True: # TODO: 生きてないときにエラー処理
      is_charged = self.serial.cd
      if is_charged == DCD_OUTPUT_BATTERY_FINE:
        return True
      if is_charged == DCD_OUTPUT_BATTERY_LOW:
        return False