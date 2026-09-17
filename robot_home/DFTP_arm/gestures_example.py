#!/usr/bin/env python3
# Управление позами Inspire Hand (номера 1-7)
# Требуется: pip install plusml-rh56dftp

from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import (
    POS_SET_0, POS_SET_1, POS_SET_2, POS_SET_3, POS_SET_4, POS_SET_5
)

import sys
import io
import time

# Настройка кодировки
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# Константы
IP = "192.168.123.211"
PORT = 6000
MOVEMENT_DELAY = 0.2  # Задержка между движениями пальцев

# Пределы движения пальцев (0 = прямой, 1800 = согнут)
FINGER_NAMES = {
    0: "Мизинец",
    1: "Безымянный", 
    2: "Средний",
    3: "Указательный",
    4: "Большой (сгиб)",
    5: "Большой (вращение)"
}

# Определение поз (номера 1-7)
POSES = {
    "1": {
        "name": "🖐️  Открытая ладонь",
        "description": "Все пальцы выпрямлены",
        "positions": [0, 0, 0, 0, 0, 0]
    },
    "2": {
        "name": "✊️  Кулак",
        "description": "Все пальцы согнуты",
        "positions": [1800, 1800, 1800, 1800, 1800, 0]
    },
    "3": {
        "name": "✌️  Победа/Мир",
        "description": "Указательный и средний пальцы вверх, остальные согнуты",
        "positions": [1800, 1800, 0, 0, 1800, 0]
    },
    "4": {
        "name": "🤘️  Рок",
        "description": "Указательный и мизинец вверх, средний и безымянный согнуты",
        "positions": [0, 1800, 1800, 0, 1800, 0]
    },
    "5": {
        "name": "👍️  Большой палец вверх",
        "description": "Большой палец поднят, остальные согнуты",
        "positions": [1800, 1800, 1800, 1800, 0, 0]
    },
    "6": {
        "name": "👆️  Указание",
        "description": "Только указательный палец вытянут",
        "positions": [1800, 1800, 1800, 0, 1800, 0]
    },
    "7": {
        "name": "👌️  OK",
        "description": "Большой и указательный формируют кольцо, остальные выпрямлены",
        "positions": [0, 0, 0, 1200, 500, 2000]

    }
}

def print_menu():
    """Вывод меню с позами"""
    print("\n" + "=" * 50)
    print("🤖 УПРАВЛЕНИЕ ПОЗАМИ INSPIRE HAND")
    print("=" * 50)
    print("\nДоступные позы:")
    
    for key in ["1", "2", "3", "4", "5", "6", "7"]:
        pose = POSES[key]
        print(f"  {key:>2}. {pose['name']}")
    
    print("\n  R. Сброс (открытая ладонь)")
    print("  X. Выход")
    print("-" * 50)

def move_finger(client, finger_id, position, finger_name):
    """Движение одного пальца"""
    print(f"  → {finger_name}: {position}")
    ok = client.set(finger_id, position)
    time.sleep(MOVEMENT_DELAY)
    return ok

def execute_pose(client, pose_key):
    """Выполнение выбранной позы"""
    if pose_key not in POSES:
        print("❌ Неизвестная поза")
        return False
    
    pose = POSES[pose_key]

    # Для "OK" сначала фиксируем геометрию большого пальца:
    # поворот -> сгиб -> указательный. Это дает более стабильную форму кольца.
    if pose_key == "7":
        print(f"\n🎭 Выполняю позу: {pose['name']}")
        print(f"📝 {pose['description']}")
        print("\nТочная установка большого пальца для OK:")

        # Открываем неучаствующие пальцы
        move_finger(client, POS_SET_0, 0, FINGER_NAMES[0])
        move_finger(client, POS_SET_1, 0, FINGER_NAMES[1])
        move_finger(client, POS_SET_2, 0, FINGER_NAMES[2])

        # Повышаем скорость большого и указательного, чтобы не "зависали" после прошлых запусков
        client.set("SPEED_SET(3)", 220)
        client.set("SPEED_SET(4)", 220)
        client.set("SPEED_SET(5)", 180)

        # Небольшой запас по усилию, чтобы большой палец доходил до позиции
        client.set("FORCE_SET(3)", 400)
        client.set("FORCE_SET(4)", 550)
        client.set("FORCE_SET(5)", 350)

        ok_index = pose["positions"][3]
        ok_thumb_flex = pose["positions"][4]
        ok_thumb_rot = pose["positions"][5]

        # 1) Поворот большого
        move_finger(client, POS_SET_5, ok_thumb_rot, FINGER_NAMES[5])
        time.sleep(0.15)
        move_finger(client, POS_SET_5, ok_thumb_rot, FINGER_NAMES[5])

        # 2) Сгиб большого
        move_finger(client, POS_SET_4, ok_thumb_flex, FINGER_NAMES[4])
        time.sleep(0.15)
        move_finger(client, POS_SET_4, ok_thumb_flex, FINGER_NAMES[4])

        # 3) Подвод указательного
        move_finger(client, POS_SET_3, ok_index, FINGER_NAMES[3])

        print(f"\n✅ Поза '{pose['name']}' установлена")
        return True
    
    print(f"\n🎭 Выполняю позу: {pose['name']}")
    print(f"📝 {pose['description']}")
    print("\nДвижение пальцев:")
    
    # Список пальцев и их регистров
    finger_registers = [POS_SET_0, POS_SET_1, POS_SET_2, POS_SET_3, POS_SET_4, POS_SET_5]
    positions = pose["positions"]
    
    # Двигаем пальцы по очереди
    for i in range(6):
        move_finger(client, finger_registers[i], positions[i], FINGER_NAMES[i])
    
    print(f"\n✅ Поза '{pose['name']}' установлена")
    return True

def reset_pose(client):
    """Сброс в открытую ладонь"""
    print("\n🔄 Сбрасываю в открытую ладонь...")
    pose = POSES["1"]
    
    finger_registers = [POS_SET_0, POS_SET_1, POS_SET_2, POS_SET_3, POS_SET_4, POS_SET_5]
    positions = pose["positions"]
    
    for i in range(6):
        move_finger(client, finger_registers[i], positions[i], FINGER_NAMES[i])
    
    print("✅ Ладонь открыта")

def main():
    """Основная функция"""
    print("🔌 Подключение к Inspire Hand...")
    
    try:
        # Подключение
        client = RH56DFTP_TCP(host=IP, port=PORT)
        print(f"✅ Подключено к {IP}:{PORT}")
        
        # Сброс в начальное положение
        reset_pose(client)
        time.sleep(1)
        
        while True:
            # Вывод меню
            print_menu()
            
            # Выбор пользователя
            choice = input("\nВыберите позу (номер) или команду: ").strip().upper()
            
            if choice == "X":
                print("\n👋 Завершение работы...")
                break
            elif choice == "R":
                reset_pose(client)
            elif choice in POSES:
                execute_pose(client, choice)
            else:
                print("❌ Неверный выбор. Попробуйте снова.")
            
            # Пауза перед следующим выбором
            time.sleep(0.5)
        
        # Закрытие соединения
        client.close()
        print("🔌 Соединение закрыто")
        
    except KeyboardInterrupt:
        print("\n\n👋 Программа прервана пользователем")
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        print(f"\n💡 Проверьте подключение к {IP}:{PORT}")
    finally:
        try:
            client.close()
        except:
            pass

if __name__ == "__main__":
    main()
