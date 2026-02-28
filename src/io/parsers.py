from datetime import time
from typing import Any, Tuple, Optional
import pandas as pd
import logging

class TimeParser:
    """Логика для времени и длительности"""
    @staticmethod
    def to_minutes(value: Any) -> int:
        if isinstance(value, time):
            return value.hour * 60 + value.minute
        if isinstance(value, str) and ":" in value:
            try:
                h, m = map(int, value.strip().split(':'))
                return h * 60 + m
            except ValueError as e:
                logging.error(f"Ошибка формата времени в строке '{value}': {e}")
            raise
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
        if pd.isna(value):
            return default
        if isinstance(value, (int, float)):
            return int(value)
        if isinstance(value, str):
            # Убираем все, кроме цифр
            cleaned = ''.join(filter(str.isdigit, value))
            return int(cleaned) if cleaned else default
        return default

    @staticmethod
    def to_float(value: Any, default: float = 0.0) -> float:
        if pd.isna(value):
            return default
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            # Оставляем цифры, точки и запятые (заменяя их на точки)
            cleaned = value.replace(',', '.').strip()
            cleaned = ''.join(c for c in cleaned if c.isdigit() or c == '.')
            try:
                return float(cleaned)
            except ValueError:
                return default
        return default