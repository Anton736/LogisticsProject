# START OF FILE src/io/base_matrix_loader.py
from abc import ABC, abstractmethod
from src.core.entities import TransportNetwork


class BaseMatrixLoader(ABC):
    """
    Абстрактный загрузчик матриц расстояний и времён.

    Наследники реализуют конкретный источник данных
    (Excel, CSV, база данных, OSRM-API и т.д.).
    """

    @abstractmethod
    def load_distance_matrix(self) -> dict:
        """
        Возвращает матрицу расстояний в формате:
            { location_id: { location_id: float } }

        Ключи — идентификаторы точек (int или str).
        Значение на диагонали равно 0.
        """

    @abstractmethod
    def load_time_matrix(self) -> dict:
        """
        Возвращает матрицу времён в формате:
            { location_id: { location_id: int } }

        Единицы измерения — минуты.
        Значение на диагонали равно 0.
        """

    def load_network(self) -> TransportNetwork:
        """
        Удобный фасадный метод: загружает обе матрицы
        и собирает готовый TransportNetwork.

        Переопределяй только если нужна нетривиальная логика сборки.
        """
        raw_dist = self.load_distance_matrix()
        raw_time = self.load_time_matrix()

        location_ids = sorted(raw_dist.keys())

        distance_matrix = [
            [raw_dist[i].get(j, 0.0) for j in location_ids]
            for i in location_ids
        ]
        time_matrix = [
            [raw_time[i].get(j, 0) for j in location_ids]
            for i in location_ids
        ]

        return TransportNetwork(
            distance_matrix=distance_matrix,
            time_matrix=time_matrix,
        )
