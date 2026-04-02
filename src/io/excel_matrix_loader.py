# START OF FILE src/io/excel_matrix_loader.py
import pandas as pd
from src.io.base_matrix_loader import BaseMatrixLoader


class ExcelMatrixLoader(BaseMatrixLoader):
    """
    Загружает матрицы расстояний и/или времён из Excel-файлов.

    Ожидаемая структура каждого листа:
    ┌──────┬────┬────┬────┐
    │  /   │  0 │  1 │  2 │  ← первая строка: ID точек (заголовки столбцов)
    ├──────┼────┼────┼────┤
    │  0   │  0 │ 12 │ 34 │  ← первый столбец: ID точек (индексы строк)
    │  1   │ 12 │  0 │ 18 │
    │  2   │ 34 │ 18 │  0 │
    └──────┴────┴────┴────┘

    Значение в ячейке (строка i, столбец j) — расстояние/время от i до j.
    Диагональ равна 0. Первая строка и первый столбец — идентификаторы точек.

    Параметры:
        distance_file:  Путь к Excel-файлу с расстояниями.
        time_file:      Путь к Excel-файлу со временем.
        distance_sheet: Имя листа (или номер) в файле расстояний.
        time_sheet:     Имя листа (или номер) в файле времён.
    """

    def __init__(
        self,
        distance_file: str,
        time_file: str,
        distance_sheet: str | int = 0,
        time_sheet: str | int = 0,
    ):
        self.distance_file = distance_file
        self.time_file = time_file
        self.distance_sheet = distance_sheet
        self.time_sheet = time_sheet

    # ------------------------------------------------------------------
    # Публичный API (реализация абстрактных методов)
    # ------------------------------------------------------------------

    def load_distance_matrix(self) -> dict:
        """Читает матрицу расстояний (км, float)."""
        return self._read_matrix(self.distance_file, self.distance_sheet, value_type=float)

    def load_time_matrix(self) -> dict:
        """Читает матрицу времён (мин, int)."""
        return self._read_matrix(self.time_file, self.time_sheet, value_type=int)

    # ------------------------------------------------------------------
    # Приватная логика разбора
    # ------------------------------------------------------------------

    @staticmethod
    def _read_matrix(file_path: str, sheet: str | int, value_type: type) -> dict:
        """
        Универсальный парсер матрицы:
          - Первый столбец → индексы строк (ID точек).
          - Первая строка  → заголовки столбцов (ID точек).
          - Тело → числовые значения, приведённые к value_type.

        Возвращает:
            { row_id: { col_id: value } }
        """
        df = pd.read_excel(file_path, sheet_name=sheet, header=0, index_col=0)

        # Нормализуем индексы и заголовки: int, если возможно
        df.index   = [ExcelMatrixLoader._normalize_id(v) for v in df.index]
        df.columns = [ExcelMatrixLoader._normalize_id(v) for v in df.columns]

        matrix: dict = {}
        for row_id in df.index:
            matrix[row_id] = {}
            for col_id in df.columns:
                raw = df.loc[row_id, col_id]
                matrix[row_id][col_id] = ExcelMatrixLoader._safe_cast(raw, value_type)

        return matrix

    @staticmethod
    def _normalize_id(value) -> int:
        """
        Приводит ID точки к int.
        Если это невозможно (например, строковый код), оставляет как есть.
        """
        try:
            return int(float(value))
        except (ValueError, TypeError):
            return value

    @staticmethod
    def _safe_cast(value, target_type: type):
        """
        Безопасное приведение к целевому типу.
        NaN → 0 (диагональ или пустые ячейки).
        """
        try:
            if pd.isna(value):
                return target_type(0)
            return target_type(float(value))
        except (ValueError, TypeError):
            return target_type(0)
