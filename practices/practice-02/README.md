# Результат выполнения практической работы 2

Для выполнения задания был создан пакет practice2 для запуска TurtleBot3 в Webots. В центре сцены размещены два препятствия в форме цифр номера варианта 08. Робот движется по окружности вокруг обеих цифр.

## Сборка 

1. Зайти в папку проекта 

2. Запустить контейнер
```bash
HOST_XAUTHORITY="$XAUTHORITY" docker compose --env-file .env --env-file .env.local -f docker-compose.yaml -f docker-compose.gpu.yaml -f docker-compose.webots-mac.yaml up -d
```

3. Запустить в нем zsh
```bash
HOST_XAUTHORITY="$XAUTHORITY" docker compose --env-file .env --env-file .env.local -f docker-compose.yaml -f docker-compose.gpu.yaml -f docker-compose.webots-mac.yaml exec -it ros2-base zsh
```

4. Собрать рабочее пространство 
```bash
cd ~/practices_ws
colcon build --symlink-install
source install/setup.zsh
```

## Запуск симулятора

Запустить launch файл
```
ros2 launch practice2 spawn_robot.launch.py
```

Запустится симулятор со сценой, на которой размещены цифры 08. 

## Запуск робота

Запустить робота по окружности:
```
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0833}}}"
```

## Видео выполнения

Видео выполнения можно посмотреть по ссылке:

[https://disk.360.yandex.ru/d/T0ETmODUNKuBUg](https://disk.360.yandex.ru/d/T0ETmODUNKuBUg)