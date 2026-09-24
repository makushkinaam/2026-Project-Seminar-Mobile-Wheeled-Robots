# Результат выполнения практической работы 1

Для выполнения задания был создан пакет practice1, содережащий управляющую программу practice1_node.py для запуска черепахи, которая рисует на экране число с использованием turtlesim. 

## Сборка 

1. Зайти в папку проекта 
2. Запустить контейнер
```bash
HOST_XAUTHORITY="$XAUTHORITY" docker compose --env-file .env --env-file .env.local -f docker-compose.yaml -f docker-compose.gpu.yaml [-f docker-compose.nvidia.yaml] up -d
```
3. Запустить в нем zsh
```bash
HOST_XAUTHORITY="$XAUTHORITY" docker compose --env-file .env --env-file .env.local -f docker-compose.yaml -f docker-compose.gpu.yaml [-f docker-compose.nvidia.yaml] exec -it ros2-base zsh
```
4. Собрать рабочее пространство 
```bash
cd ~/practices_ws
colcon build --symlink-install
source install/setup.zsh
```

## Запуск 

Запустить launch файл с параметрами для двух черепах
```
ros2 launch practice1 practice1.launch.py turtle_name1:=turtle2 turtle_name2:=turtle3 num1:=0 num2:=8
```

Черепаха нарисует цифры 08

## Граф узлов

Просмотреть граф узлов:
```
rqt_graph
```

## Видео выполнения

Видео выполнения можно посмотреть по ссылке:

[https://disk.360.yandex.ru/d/YLtclqlWGW7FyA](https://disk.360.yandex.ru/d/YLtclqlWGW7FyA)




