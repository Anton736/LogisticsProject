# START OF FILE enums.py
from enum import Enum

class WarehouseCostMode(Enum):
    """Режимы расчета переменной w_iv (пиковый/обработанный объем склада)."""
    PEAK_INPUT = 1      # Вариант A: Начальный запас + все поступления (более точная верхняя оценка)
    EXACT_PEAK = 2      # Вариант B: Точный пиковый запас (для будущей, сложной реализации)