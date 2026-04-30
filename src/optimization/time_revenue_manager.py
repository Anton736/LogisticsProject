from abc import ABC, abstractmethod
from typing import Dict, List
from ortools.constraint_solver import pywrapcp

from src.core.entities import Store


class BaseTimeRevenueManager(ABC):
    """
    Абстрактный класс для управления зависимостью выручки от времени прибытия.
    Позволяет легко подменять логику расчета без изменения самого алгоритма.
    """

    @abstractmethod
    def compute_revenue(self, store: Store, arrival_minute: int, base_revenue: float) -> float:
        pass
    @abstractmethod
    def apply_penalties(self,
                        manager: pywrapcp.RoutingIndexManager,
                        routing: pywrapcp.RoutingModel,
                        time_dim: pywrapcp.RoutingDimension,
                        all_locs: List[any],
                        loc_idx: Dict[str, int],
                        node_revenue: Dict[str, float],
                        scale: int):
        """
        Метод, который должен применять штрафы к размерности времени (time_dim).
        """
        pass


class LinearTimeRevenueManager(BaseTimeRevenueManager):
    """
    Реализует линейное падение цены.
    Максимальная цена на старте окна (например, в 9:00).
    К концу окна (например, в 12:00) цена падает до указанного процента (по умолчанию 0).
    До старта окна - цена максимальна (ожидание не штрафуется выручкой).
    После конца окна - жесткий запрет (контролируется основным кодом).
    """

    def __init__(self, drop_to_percent: float = 0.0):
        """
        :param drop_to_percent: До какого процента падает выручка к концу окна (0.0 = до нуля).
        """
        self.drop_to_percent = max(0.0, min(1.0, drop_to_percent))

    def apply_penalties(self,
                        manager: pywrapcp.RoutingIndexManager,
                        routing: pywrapcp.RoutingModel,
                        time_dim: pywrapcp.RoutingDimension,
                        all_locs: List[any],
                        loc_idx: Dict[str, int],
                        node_revenue: Dict[str, float],
                        scale: int):

        for loc in all_locs:
            if isinstance(loc, Store):
                idx = manager.NodeToIndex(loc_idx[loc.id])

                # Исходная максимальная выручка (уже в масштабе)
                max_rev = node_revenue.get(loc.id, 0.0) * scale
                if max_rev <= 0:
                    continue

                window_length = loc.time_end - loc.time_start
                if window_length <= 0:
                    continue

                # Сколько выручки мы теряем за все окно
                lost_revenue_total = max_rev * (1.0 - self.drop_to_percent)

                # Штраф за каждую минуту опоздания после time_start
                penalty_per_minute = int(lost_revenue_total / window_length)

                if penalty_per_minute > 0:
                    # Встроенный быстрый метод OR-Tools:
                    # Если время > time_start, за каждую 1 ед. времени вычитай penalty_per_minute
                    time_dim.SetCumulVarSoftUpperBound(idx, loc.time_start, penalty_per_minute)

    def compute_revenue(self, store: Store, arrival_minute: int, base_revenue: float) -> float:
        window_length = store.time_end - store.time_start
        if window_length <= 0 or arrival_minute <= store.time_start:
            return base_revenue
        if arrival_minute >= store.time_end:
            return base_revenue * self.drop_to_percent
        progress = (arrival_minute - store.time_start) / window_length
        multiplier = 1.0 - progress * (1.0 - self.drop_to_percent)
        return base_revenue * multiplier