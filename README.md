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

Заходим в папку проекта и запускаем проект с помощью `launch.sh`. Внутри bash-файла собирается проект с помощью colcon build и запускается нода для управления моторами.

```
cd Challenge-Tinker
bash launch.sh
```
