# START OF FILE objective_builder.py
from ortools.sat.python import cp_model
from typing import Tuple

from src.core.entities import Scenario  # Добавляем Warehouse для явного использования
from src.optimization.var_manager import VarManager



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
        Строит выражения для числителя и знаменателя с реалистичными границами int64.
        """
        total_cost_numerator_terms = []

        # Предварительный расчет для границ: общая емкость флота и макс. время суток
        total_fleet_capacity = sum(v.capacity for v in self.scenario.vehicles)
        MAX_DAY_MINUTES = 1440  # 24 часа
        MAX_POSSIBLE_KM = 3000  # Запас хода, который машина физически не превысит за день

        # 1. Затраты на транспорт
        for v in self.scenario.vehicles:
            vehicle_used = self.var_manager.get_vehicle_used_var(v.id)
            total_dist = self.var_manager.get_total_dist_var(v.id)
            total_time = self.var_manager.get_total_time_var(v.id)

            # РАССЧИТЫВАЕМ РЕАЛЬНЫЙ МАКСИМУМ для одной машины
            # (Подача + макс_км * цена_км + 24ч * цена_часа) * запас 20%
            v_max_cost_raw = (v.cost_call + (v.cost_km * MAX_POSSIBLE_KM) + (v.cost_hour * MAX_DAY_MINUTES))
            v_logical_max = int(v_max_cost_raw * 1.2 * self.scale_factor)

            vehicle_cost_var = self.model.new_int_var(0, v_logical_max, f'cost_v{v.id}')

            cost_call_scaled = self._scale(v.cost_call)
            cost_km_scaled = self._scale(v.cost_km)
            cost_hour_scaled = self._scale(v.cost_hour)

            cost_expr = cost_call_scaled * vehicle_used + cost_km_scaled * total_dist + cost_hour_scaled * total_time

            self.model.add(vehicle_cost_var == cost_expr).OnlyEnforceIf(vehicle_used)
            self.model.add(vehicle_cost_var == 0).OnlyEnforceIf(vehicle_used.Not())

            total_cost_numerator_terms.append(vehicle_cost_var)

        # 2. Затраты на склады
        for wh in self.scenario.warehouses:
            wh_active = self.var_manager.get_wh_active_var(wh.id)
            wh_max_vol = self.var_manager.get_wh_max_vol_var(wh.id)

            # РАССЧИТЫВАЕМ РЕАЛЬНЫЙ МАКСИМУМ для склада
            # (фикс_затраты + цена_ед * весь_объем_всех_машин) * запас 20%
            wh_max_cost_raw = (wh.fixed_staff_cost + (wh.cost_per_volume * total_fleet_capacity))
            wh_logical_max = int(wh_max_cost_raw * 1.2 * self.scale_factor)

            warehouse_cost_var = self.model.new_int_var(0, wh_logical_max, f'cost_wh{wh.id}')

            cost_per_vol_scaled = self._scale(wh.cost_per_volume)
            fixed_cost_scaled = self._scale(wh.fixed_staff_cost)

            cost_expr = cost_per_vol_scaled * wh_max_vol + fixed_cost_scaled

            self.model.add(warehouse_cost_var == cost_expr).OnlyEnforceIf(wh_active)
            self.model.add(warehouse_cost_var == 0).OnlyEnforceIf(wh_active.Not)

            total_cost_numerator_terms.append(warehouse_cost_var)

        numerator_expr = sum(total_cost_numerator_terms)

        # --- Знаменатель ---
        total_delivered_volume_terms = []
        for v in self.scenario.vehicles:
            for store in self.scenario.stores:
                for brand in self.scenario.brands:
                    total_delivered_volume_terms.append(self.var_manager.get_delivery_var(v.id, store.id, brand.id))

        total_delivered_volume = sum(total_delivered_volume_terms)

        # Здесь тоже важен логический лимит для знаменателя, чтобы не уйти в бесконечность
        # (цена за ящик берется из сценария)
        bread_unit_cost_scaled = self._scale(self.scenario.bread_unit_cost)
        denominator_expr = total_delivered_volume * bread_unit_cost_scaled

        return numerator_expr, denominator_expr

# END OF FILE objective_builder.py