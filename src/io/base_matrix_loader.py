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
        raw_dist = self.load_distance_matrix()
        raw_time = self.load_time_matrix()

        location_ids = list(raw_dist.keys())

        distance_matrix = {}
        time_matrix = {}

        for i in location_ids:
            distance_matrix[i] = {}
            time_matrix[i] = {}
            for j in location_ids:
                # Читаем базовые значения (если нет, ставим 0)
                dist = raw_dist[i].get(j, 0.0)
                base_time = raw_time[i].get(j, 0)

                # ЛОГИКА РАЗГОНА И ТОРМОЖЕНИЯ
                if i == j:
                    # Машина никуда не едет (сама к себе), время 0
                    final_time = 0
                else:
                    # dist измеряется в км. 50 метров = 0.05 км
                    if dist <= 0.05:
                        final_time = base_time + 2
                    else:
                        final_time = base_time + 5

                distance_matrix[i][j] = dist
                time_matrix[i][j] = final_time

        return TransportNetwork(
            distance_matrix=distance_matrix,
            time_matrix=time_matrix,
        )
