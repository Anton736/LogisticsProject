# START OF FILE objective_builder.py
from ortools.sat.python import cp_model
from typing import Tuple, List

from entities import Scenario, Store, Warehouse  # Добавляем Warehouse для явного использования
from var_manager import VarManager



class ObjectiveBuilder:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario, var_manager: VarManager, scale_factor: int = 1000):
        self.model = model
        self.scenario = scenario
        self.var_manager = var_manager
        self.scale_factor = scale_factor

    def _scale(self, value: float) -> int:
        """Вспомогательная функция для масштабирования float в int."""
        return int(value * self.scale_factor)

    def build_objective_expressions(self) -> Tuple[cp_model.LinearExpr, cp_model.LinearExpr]:
        """
        Строит выражения для числителя (общие затраты) и знаменателя (общая стоимость доставленного товара),
        обеспечивая целочисленную арифметику.
        """
        # --- Числитель: Общие затраты ---
        total_cost_numerator_terms = []

        # 1. Затраты на использование транспортных средств
        for v in self.scenario.vehicles:
            vehicle_used = self.var_manager.get_vehicle_used_var(v.id)
            total_dist = self.var_manager.get_total_dist_var(v.id)
            total_time = self.var_manager.get_total_time_var(v.id)

            vehicle_cost_var = self.model.NewIntVar(0, 10_000_000_000 * self.scale_factor, f'cost_v{v.id}')

            cost_call_scaled = self._scale(v.cost_call)
            cost_km_scaled = self._scale(v.cost_km)
            cost_hour_scaled = self._scale(v.cost_hour)

            cost_expr = cost_call_scaled + cost_km_scaled * total_dist + cost_hour_scaled * total_time

            self.model.Add(vehicle_cost_var == cost_expr).OnlyEnforceIf(vehicle_used)
            self.model.Add(vehicle_cost_var == 0).OnlyEnforceIf(self.model.Not(vehicle_used))

            total_cost_numerator_terms.append(vehicle_cost_var)

        # 2. Затраты на использование складов/РЦ
        for wh in self.scenario.warehouses:
            wh_active = self.var_manager.get_wh_active_var(wh.id)
            # ObjectiveBuilder просто использует wh_max_vol_var, не зная, как она была вычислена
            wh_max_vol = self.var_manager.get_wh_max_vol_var(wh.id)

            warehouse_cost_var = self.model.NewIntVar(0, 10_000_000_000 * self.scale_factor, f'cost_wh{wh.id}')

            cost_per_vol_scaled = self._scale(wh.cost_per_volume)
            fixed_cost_scaled = self._scale(wh.fixed_staff_cost)

            cost_expr = cost_per_vol_scaled * wh_max_vol + fixed_cost_scaled

            self.model.Add(warehouse_cost_var == cost_expr).OnlyEnforceIf(wh_active)
            self.model.Add(warehouse_cost_var == 0).OnlyEnforceIf(self.model.Not(wh_active))

            total_cost_numerator_terms.append(warehouse_cost_var)

        numerator_expr = sum(total_cost_numerator_terms)

        # --- Знаменатель: Общая стоимость доставленного товара ---
        total_delivered_volume_terms = []
        for v in self.scenario.vehicles:
            for store in self.scenario.stores:
                for brand in self.scenario.brands:
                    # m_nt'vk0 - объем бренда b который привез k водитель в n магазин во время t'
                    total_delivered_volume_terms.append(self.var_manager.get_delivery_var(v.id, store.id, brand.id))

        total_delivered_volume = sum(total_delivered_volume_terms)

        bread_unit_cost_scaled = self._scale(self.scenario.bread_unit_cost)
        denominator_expr = total_delivered_volume * bread_unit_cost_scaled

        return numerator_expr, denominator_expr

# END OF FILE objective_builder.py