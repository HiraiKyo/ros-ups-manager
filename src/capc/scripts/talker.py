import rospy
from std_msgs.msg import Float32 as Float
from serials.cabinet import Cabinet_serial
from serials.ups import Ups_serial

# Class Initialize
cabinet = Cabinet_serial()
ups = Ups_serial()

def cb_cyclic(ev):
  print("watchdog::cyclic call back")
  t1=Float()
  t1.data = cabinet.get_temperature()
  rospy.set_param("/watchdog/inner_temp",t1.data)


def talker():
  pub=rospy.Publisher('/watchdog/inner_temp',Float,queue_size=1)
  rospy.init_node('talker', anonymous=True)
  
  # TODO: Config loader
  # try:
  #   Config.update(rospy.get_param("/config/watchdog"))
  # except Exception as e:
  #   print("get_param exception:",e.args)
  
  # Connection
  cabinet.connect()
  ups.connect()
  
  rospy.Timer(rospy.Duration(2),cb_cyclic)

  while not rospy.is_shutdown():
    print("watchdog::main loop")

    # TODO: シリアル通信の再接続処理
    
    # バッテリーモードへの変更を検知してシャットダウン処理を実行
    mode = ups.is_battery_mode()
    if mode == True:
      ups.shutdown()

    # バッテリー残量低下をキャビネット前面に通知
    is_low = ups.is_battery_fine()
    cabinet.set_battery_state(is_low)
    
    # シャットダウンスクリプトが生存しているかをキャビネット前面に通知
    is_alive = True # TODO: スクリプトが実行状態かどうかの判定方法
    cabinet.set_shutdowner_state(is_alive)
    rospy.sleep(1)
    
  # FIXME: ROSが落ちたらシリアル接続も解除する？　その場合はここ？
  # ups.disconnect()
  # cabinet.disconnect()


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException: pass