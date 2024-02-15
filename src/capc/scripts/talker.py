import rospy
from std_msgs.msg import Float32 as Float
from serials.cabinet import Cabinet_serial
from serials.ups import Ups_serial

class main():
  def __init__(self):
    self.cabinet = Cabinet_serial()
    self.ups = Ups_serial()
    self.pub = rospy.Publisher('/watchdog/inner_temp',Float,queue_size=1)
    self.talker()
    
  def cb_cyclic(self, ev):
    print("watchdog::cyclic call back")
    t1=Float()
    t1.data = self.cabinet.get_temperature()
    self.pub.publish(t1)
    rospy.set_param("/watchdog/inner_temp",t1.data)

  def talker(self):
    rospy.init_node('talker', anonymous=True)
    
    # TODO: Config loader
    # try:
    #   Config.update(rospy.get_param("/config/watchdog"))
    # except Exception as e:
    #   print("get_param exception:",e.args)
    
    # Connection
    self.cabinet.connect()
    self.ups.connect()
    
    rospy.Timer(rospy.Duration(2), self.cb_cyclic)

    while not rospy.is_shutdown():
      print("watchdog::main loop")

      # TODO: シリアル通信の再接続処理
      
      # バッテリーモードへの変更を検知してシャットダウン処理を実行
      mode = self.ups.is_battery_mode()
      if mode == True:
        self.ups.shutdown()

      # バッテリー残量低下をキャビネット前面に通知
      is_low = self.ups.is_battery_fine()
      self.cabinet.set_battery_state(is_low)
      
      # シャットダウンスクリプトが生存しているかをキャビネット前面に通知
      is_alive = True # TODO: スクリプトが実行状態かどうかの判定方法
      self.cabinet.set_shutdowner_state(is_alive)
      rospy.sleep(1)
      
    # FIXME: ROSが落ちたらシリアル接続も解除する？　その場合はここ？
    # self.dispose()

  def dispose(self):
    self.ups.disconnect()
    self.cabinet.disconnect()
    # FIXME: ROSまわりでデストラクタ処理必要？

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass