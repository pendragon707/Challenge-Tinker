# Челлендж Tinker

## Сборка проекта

### 1. Установка зависимостей

Установите на Raspberry Pi образ Ubuntu Server 24.04 и ros2 jazzy.

```
source /opt/ros/jazzy/setup.bash
```

### 2. Сборка проекта

```
cd ~
git clone git@github.com:pendragon707/Challenge-Tinker.git
```

Заходим в папку проекта и запускаем сборку с помощью colcon. Эта команда компилирует все пакеты ROS2, находящиеся в рабочем пространстве.
```
cd Challenge-Tinker
colcon build
source install/setup.bash
```
Запускает указанный исполняемую ноду из пакета. Убедитесь, что вы выполнили `source install/setup.bash` после сборки, чтобы команды ROS2 могли найти ваши пакеты. 
```
ros2 run ...
```
