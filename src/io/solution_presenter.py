from typing import List, Dict
from src.core.entities import Scenario, Vehicle, Location, VehicleAssignment, WarehouseAssignment
from src.models.solution import Solution


class SolutionPresenter:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.location_by_id: Dict[int, Location] = {loc.id: loc for loc in scenario.all_locations}

    def build_solution(self, solver, var_manager, optimal_lambda,
                       numerator_value_scaled, denominator_value_scaled,
                       objective_scale_factor) -> Solution:

        vehicle_assignments = []
        for v in self.scenario.vehicles:
            # Используем safe_value, чтобы не вызвать ошибку если переменной нет
            is_active = False
            try:
                is_active = solver.BooleanValue(var_manager.get_vehicle_used_var(v.id))
            except:
                pass

            if is_active:
                route = self._reconstruct_route(solver, var_manager, v)
                vehicle_assignments.append(VehicleAssignment(
                    vehicle=v,
                    route=route,
                    total_time=solver.Value(var_manager.get_total_time_var(v.id)),
                    total_dist=solver.Value(var_manager.get_total_dist_var(v.id)),
                    is_active=True
                ))
            else:
                vehicle_assignments.append(VehicleAssignment(vehicle=v, route=[], is_active=False))

        warehouse_assignments = []
        for wh in self.scenario.warehouses:
            is_active = solver.BooleanValue(var_manager.get_wh_active_var(wh.id))
            max_vol = solver.Value(var_manager.get_wh_max_vol_var(wh.id))
            warehouse_assignments.append(WarehouseAssignment(
                warehouse=wh, is_active=is_active, max_volume=max_vol
            ))

        return Solution(
            vehicle_assignments=vehicle_assignments,
            warehouse_assignments=warehouse_assignments,
            optimal_objective_value=optimal_lambda,
            total_numerator_cost=numerator_value_scaled / objective_scale_factor,
            total_denominator_value=denominator_value_scaled / objective_scale_factor
        )

    def _reconstruct_route(self, solver, var_manager, vehicle: Vehicle) -> List[Location]:
        route = []
        # Находим склад отправления
        curr_loc_id = -1
        for wh in self.scenario.warehouses:
            for j_loc in self.scenario.all_locations:
                x_var = var_manager.get_routing_var(vehicle.id, wh.id, j_loc.id)
                if x_var and solver.BooleanValue(x_var):
                    curr_loc_id = wh.id
                    break
            if curr_loc_id != -1: break

        if curr_loc_id == -1: return []

        visited_ids = set()
        max_len = len(self.scenario.all_locations) + 1

        for _ in range(max_len):
            if curr_loc_id in visited_ids and curr_loc_id in [w.id for w in self.scenario.warehouses]:
                # Дошли до склада повторно (завершили маршрут)
                route.append(self.location_by_id[curr_loc_id])
                break

            visited_ids.add(curr_loc_id)
            route.append(self.location_by_id[curr_loc_id])

            next_loc_id = -1
            for j_loc in self.scenario.all_locations:
                if j_loc.id == curr_loc_id: continue
                x_var = var_manager.get_routing_var(vehicle.id, curr_loc_id, j_loc.id)
                if x_var and solver.BooleanValue(x_var):
                    next_loc_id = j_loc.id
                    break

            if next_loc_id == -1: break  # Тупик
            curr_loc_id = next_loc_id

        return route