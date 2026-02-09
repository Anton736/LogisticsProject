# START OF FILE solution_presenter.py
from ortools.sat.python import cp_model
from typing import List, Dict, Tuple, Any

from entities import Scenario, Vehicle, Store, Warehouse, VehicleAssignment, WarehouseAssignment, Location
from var_manager import VarManager
from solution import Solution


class SolutionPresenter:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.location_by_id: Dict[int, Location] = {loc.id: loc for loc in scenario.all_locations}

    def _get_vehicle_route(self, solver: cp_model.CpSolver, var_manager: VarManager, v: Vehicle) -> List[Location]:
        """Восстанавливает маршрут для одной машины."""
        route_locations: List[Location] = []

        # Определяем, откуда машина начала свой маршрут (из какого склада)
        start_location_id = -1
        for wh in self.scenario.warehouses:
            # Ищем исходящие дуги из склада
            for j_loc in self.scenario.all_locations:
                x_var = var_manager.get_routing_var(v.id, wh.id, j_loc.id)
                if x_var and solver.BooleanValue(x_var):
                    start_location_id = wh.id
                    break
            if start_location_id != -1:
                break

        if start_location_id == -1:  # Машина не использовалась или не имеет начальной точки
            return []

        current_location_id = start_location_id
        route_locations.append(self.location_by_id[current_location_id])

        # Строим маршрут, пока не вернемся на склад или не закончим
        visited_count = 0
        max_visits_safety_limit = len(self.scenario.all_locations) + 2  # Защита от бесконечного цикла

        while True:
            next_location_id = -1
            found_next = False
            for j_loc in self.scenario.all_locations:
                if current_location_id == j_loc.id: continue  # Не идем в ту же точку
                x_var = var_manager.get_routing_var(v.id, current_location_id, j_loc.id)
                if x_var and solver.BooleanValue(x_var):
                    next_location_id = j_loc.id
                    found_next = True
                    break

            if found_next:
                if next_location_id == start_location_id:  # Вернулись на начальный склад
                    route_locations.append(self.location_by_id[next_location_id])
                    break

                route_locations.append(self.location_by_id[next_location_id])
                current_location_id = next_location_id
                visited_count += 1
                if visited_count >= max_visits_safety_limit:
                    print(
                        f"Warning: Route for vehicle {v.id} exceeded safety limit {max_visits_safety_limit}. Possible cycle or error. Route: {[loc.name for loc in route_locations]}")
                    break
            else:
                break  # Маршрут закончился (машина остановилась где-то, что не является складом - не должно быть при корректных ограничениях)

        return route_locations

    def build_solution(self,
                       solver: cp_model.CpSolver,
                       var_manager: VarManager,
                       optimal_lambda: float,
                       numerator_value_scaled: int,
                       denominator_value_scaled: int,
                       objective_scale_factor: int) -> Solution:
        """
        Извлекает значения переменных из решенной модели и формирует объект Solution.
        """
        vehicle_assignments: List[VehicleAssignment] = []
        warehouse_assignments: List[WarehouseAssignment] = []

        # Извлечение результатов по машинам
        for v in self.scenario.vehicles:
            is_active = solver.BooleanValue(var_manager.get_vehicle_used_var(v.id))
            total_dist = solver.Value(var_manager.get_total_dist_var(v.id))
            total_time = solver.Value(var_manager.get_total_time_var(v.id))

            route: List[Location] = []  # Храним объекты Location, а не только ID
            if is_active:
                route = self._get_vehicle_route(solver, var_manager, v)

            assignment = VehicleAssignment(
                vehicle=v,
                route=[loc.id for loc in route],  # В VehicleAssignment храним только ID для простоты
                total_time=total_time,
                total_dist=total_dist,
                is_active=is_active
            )
            vehicle_assignments.append(assignment)

        # Извлечение результатов по складам
        for wh in self.scenario.warehouses:
            is_active = solver.BooleanValue(var_manager.get_wh_active_var(wh.id))
            max_volume = solver.Value(var_manager.get_wh_max_vol_var(wh.id))  # w_iv

            assignment = WarehouseAssignment(
                warehouse=wh,
                is_active=is_active,
                max_volume=max_volume
            )
            warehouse_assignments.append(assignment)

        # Пересчет масштабированных значений в реальные для итогового отображения
        total_numerator_cost_real = numerator_value_scaled / objective_scale_factor
        total_denominator_value_real = denominator_value_scaled / objective_scale_factor  # Это уже реальная ценность (volume * cost)

        return Solution(
            vehicle_assignments=vehicle_assignments,
            warehouse_assignments=warehouse_assignments,
            optimal_objective_value=optimal_lambda,
            total_numerator_cost=total_numerator_cost_real,
            total_denominator_value=total_denominator_value_real
        )

    def build_vehicle_route(self, solver, var_manager, vehicle):
        """Метод восстанавливает цепочку посещений из бинарных переменных x_kij"""
        route = []
        # Находим стартовый склад (откуда выходит дуга x=1)
        current_loc = self._find_start_depot(solver, var_manager, vehicle)

        while current_loc is not None:
            # Снимаем показания в этой точке
            arrival = solver.Value(var_manager.arrival_times[(vehicle.id, current_loc.id)])
            load = solver.Value(var_manager.load_at_point[(vehicle.id, current_loc.id)])

            route.append({
                "name": current_loc.name,
                "arrival_time": self._format_time(arrival),
                "load_after": load
            })

            # Ищем следующую точку
            next_loc = None
            for j_id in self.location_ids:
                x_var = var_manager.x.get((vehicle.id, current_loc.id, j_id))
                if x_var and solver.BooleanValue(x_var):
                    next_loc = self.location_by_id[j_id]
                    break

            # Если вернулись на склад — маршрут окончен
            if isinstance(next_loc, Warehouse) or next_loc is None:
                if next_loc:
                    route.append({"name": next_loc.name, "arrival_time": "Finish"})
                break
            current_loc = next_loc

        return route