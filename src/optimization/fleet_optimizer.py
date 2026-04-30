# START OF FILE fleet_optimizer.py
from typing import Optional
from src.core.entities import Scenario
from src.optimization.pruner import RoutePruner
from src.models.demand import DemandManager
from src.core.enums import WarehouseCostMode
from src.optimization.routing_orchestrator import RoutingOrchestrator
from src.optimization.time_revenue_manager import BaseTimeRevenueManager
from src.models.solution import Solution


class FleetOptimizer:
    """
    Перебирает заданный диапазон размеров флота, запускает RoutingOrchestrator
    для каждого N и возвращает решение с максимальной чистой прибылью.
    """

    def __init__(self,
                 scenario: Scenario,
                 pruner: RoutePruner,
                 demand_manager: DemandManager,
                 min_vehicles: Optional[int] = None,
                 max_vehicles: Optional[int] = None,
                 warehouse_cost_mode: WarehouseCostMode = WarehouseCostMode.PEAK_INPUT,
                 time_manager: Optional[BaseTimeRevenueManager] = None,
                 time_limit_per_run: int = 900):
        """
        :param min_vehicles: Минимум машин (по умолчанию 1)
        :param max_vehicles: Максимум машин (по умолчанию весь флот)
        :param time_limit_per_run: Секунд на один прогон (по умолчанию 15 мин)
        """
        self.scenario = scenario
        self.pruner = pruner
        self.demand_manager = demand_manager
        self.warehouse_cost_mode = warehouse_cost_mode
        self.time_manager = time_manager
        self.time_limit_per_run = time_limit_per_run

        self.min_vehicles = min_vehicles if min_vehicles is not None else 1
        self.max_vehicles = max_vehicles if max_vehicles is not None else len(scenario.vehicles)

    def optimize(self) -> Optional[Solution]:
        best_solution: Optional[Solution] = None
        best_profit = float('-inf')

        total = self.max_vehicles - self.min_vehicles + 1
        print(f"\n=== FleetOptimizer: перебор от {self.min_vehicles} до {self.max_vehicles} машин ({total} прогонов) ===")

        for n in range(self.min_vehicles, self.max_vehicles + 1):
            print(f"\n>>> Прогон N={n} машин")

            # Обрезаем флот до N машин
            trimmed_scenario = self._trim_scenario(n)

            orchestrator = RoutingOrchestrator(
                scenario=trimmed_scenario,
                pruner=self.pruner,
                demand_manager=self.demand_manager,
                warehouse_cost_mode=self.warehouse_cost_mode,
                time_manager=self.time_manager,
                time_limit_per_run=self.time_limit_per_run,
            )

            solution = orchestrator.solve()
            if solution is None:
                print(f"  N={n}: решение не найдено, пропускаем.")
                continue

            profit = solution.total_denominator_value - solution.total_numerator_cost
            print(f"  N={n}: прибыль={profit:.2f} (выручка={solution.total_denominator_value:.2f}, затраты={solution.total_numerator_cost:.2f})")

            if profit > best_profit:
                best_profit = profit
                best_solution = solution
                print(f"  >>> Новый лучший результат при N={n}")

        if best_solution:
            print(f"\n=== Лучший флот: прибыль={best_profit:.2f} ===")
        else:
            print("\n=== FleetOptimizer: ни одного решения не найдено ===")

        return best_solution

    def _trim_scenario(self, n: int) -> Scenario:
        """Возвращает копию сценария с первыми N машинами."""
        from dataclasses import replace
        trimmed_vehicles = self.scenario.vehicles[:n]
        return replace(self.scenario, vehicles=trimmed_vehicles)

# END OF FILE fleet_optimizer.py