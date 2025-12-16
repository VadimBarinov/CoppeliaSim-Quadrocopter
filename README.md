# Quadrocopter
Поиск объекта квадрокоптером

---

- Используется Legacy RemoteAPI для взаимодействия с CoppeliaSim
- Основной код написан на Python
- `/schene/challenge_scenario.ttt` файл со сценой для CoppeliaSim

---

# Запуск проекта

1. Склонировать данный репозиторий
```bash
git clone https://github.com/VadimBarinov/CoppeliaSim-Quadrocopter.git
```

2. Перейти в директорию с проектом
```bash
cd CoppeliaSim-Quadrocopter
```

2. Открыть сцену `/schene/challenge_scenario.ttt` в CoppeliaSim

3. Загрузить необходимые библиотеки
```bash
pip install poetry
```
```bash
poetry install
```
3. Запустить Python скрипт 
```bash
poetry run run_quadrocopter
```

---
