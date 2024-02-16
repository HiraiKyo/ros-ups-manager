# ros-ups-manager

UPS Power Unit Manager for Nipron, on ROS System. (ROS1 Noetic)

# Docker Environments for development

`docker-compose up -d --build`

# 仕様

- `/watchdog/inner_temp`で内部温度を購読可能
- シャットダウン処理実行
- バッテリー残量低下で Arduino 側に異常ランプ点灯指示
- シャットダウンスクリプトの正常稼働で Arduino 側の異常ランプ消灯指示

# 非機能要件

- シリアル通信の再接続処理

# データプロトコル

## Arduino( from PC )

| key | values | params |
| battery | 0, 1 | 0: 点滅, 1: 点灯 |
| error | 0, 1, 2 | 0: 消灯, 1: 点灯, 2: 点灯 |

### Format example

`battery=1`

## Arduino( to PC )

| key | values | params |
| inner_temp | float | 0: default param |

### Format example

`inner_temp=22.1231245`

## TODO:

- Talker を Ctrl+c で終了したのちに再度`roslaunch`を実行すると、シリアル接続時にタイムアウトエラーになる。
- 自動シャットダウンが実行されない
