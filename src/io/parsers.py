from datetime import time
from typing import Any, Tuple
import pandas as pd
import logging

class TimeParser:
    """Логика для времени и длительности"""
    @staticmethod
    def to_minutes(value: Any) -> int:
        if pd.isna(value): return 0
        if isinstance(value, time):
            return value.hour * 60 + value.minute
        if isinstance(value, (int, float)): # Если время пришло как доля суток (Excel формат)
            return int(value * 1440)
        if isinstance(value, str) and ":" in value:
            try:
                parts = value.strip().split(':')
                h = int(parts[0])
                m = int(parts[1]) if len(parts) > 1 else 0
                return h * 60 + m
            except (ValueError, IndexError):
                logging.warning(f"Некорректный формат времени: '{value}'. Возвращаем 0.")
        return 0

class CoordinateParser:
    """Логика для обработки координат"""
    @staticmethod
    def parse(value: Any) -> Tuple[float, float]:
        if not isinstance(value, str): return 0.0, 0.0
        try:
            parts = value.replace(' ', '').split(',')
            return float(parts[0]), float(parts[1])
        except ValueError as e:
            logging.error(f"Ошибка формата координат в строке '{value}': {e}")
        raise

class RateExtractor:
    """Логика поиска ставок в справочнике"""
    def __init__(self, df_ref: pd.DataFrame, mapping):
        self.df = df_ref
        self.mapping = mapping

    def get_float_value(self, label: str) -> float:
        # Ищем строку, где в первом столбце есть ключевое слово (например, "Ставка")
        mask = self.df.iloc[:, 0].astype(str).str.contains(label, na=False)
        row = self.df[mask]
        if not row.empty:
            val = row.iloc[0, 1]
            if isinstance(val, str):
                # Извлекаем только цифры и точки (чтобы убрать "руб.", "км" и т.д.)
                return float(''.join(c for c in val if c.isdigit() or c in '.-'))
            return float(val)
        return 0.0
class NumericParser:
    """Очистка и конвертация чисел из грязных строк Excel"""
    @staticmethod
    def to_int(value: Any, default: int = 0) -> int:
        if pd.isna(value) or value == '': return default
        try:
            # Сначала в float, потом в int (обработка "123.0")
            return int(float(value))
        except (ValueError, TypeError):
            # Если это строка с мусором, пробуем вытащить число
            if isinstance(value, str):
                cleaned = ''.join(c for c in value if c.isdigit() or c == '.' or c == ',')
                try:
                    return int(float(cleaned.replace(',', '.')))
                except: pass
            logging.warning(f"Не удалось преобразовать в int: {value}")
            return default

    @staticmethod
    def to_float(value: Any, default: float = 0.0) -> float:
        if pd.isna(value) or value == '': return default
        try:
            if isinstance(value, str):
                value = value.replace(',', '.').strip()
            return float(value)
        except (ValueError, TypeError):
            logging.warning(f"Не удалось преобразовать в float: {value}")
            return default