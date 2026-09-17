# Сначала установите библиотеку: pip install plusml-rh56dftp
from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP
from Register.RegisterKey.ftp_registers_keys import *

# Инициализация подключения к тактильной руке
try:
    # Замените на IP-адрес и порт вашего устройства
    client = RH56DFTP_TCP(host="192.168.123.211", port=6000)
    print("✅ Успешное подключение к тактильной руке")
    
    # 1. Использование формы объекта функции для доступа к регистрам (рекомендуется, поддерживает автодополнение в IDE)
    print("\n1. Использование формы объекта функции:")
    hand_id = client.get(HAND_ID)
    print(f"🤖 ID устройства: {hand_id}")
    
    # Запись в регистр с использованием объекта функции
    success = client.set(HAND_ID, 2)
    print(f"🔧 Установка HAND_ID в 2: {success}")
    
    # Чтение значений усилий пальцев - с использованием объекта функции
    print("\n💪 Значения усилий (г):")
    print(f"   - Мизинец: {client.get(FORCE_ACT_0)} г")
    print(f"   - Безымянный палец: {client.get(FORCE_ACT_1)} г")
    print(f"   - Средний палец: {client.get(FORCE_ACT_2)} г")
    print(f"   - Указательный палец: {client.get(FORCE_ACT_3)} г")
    print(f"   - Сгибание большого пальца: {client.get(FORCE_ACT_4)} г")
    print(f"   - Вращение большого пальца: {client.get(FORCE_ACT_5)} г")
    
    # 2. Использование строковой формы для доступа к регистрам (совместимость со старыми версиями)
    print("\n2. Использование строковой формы:")
    hand_id_str = client.get("HAND_ID")
    print(f"🤖 ID устройства (строковая форма): {hand_id_str}")
    
    # Чтение значений усилий в строковой форме
    force_0_str = client.get("FORCE_ACT(0)")
    temp_1_str = client.get("TEMP(1)")
    print(f"💪 Усилие мизинца (строковая форма): {force_0_str} г")
    print(f"🌡️ Температура исполнительного механизма 1 (строковая форма): {temp_1_str} °C")
    
    # Закрытие соединения
    client.close()
    print("\n👋 Соединение закрыто")
except Exception as e:
    print(f"❌ Ошибка: {e}")
