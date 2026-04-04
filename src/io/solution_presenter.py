from typing import List, Dict
from src.core.entities import Scenario, Vehicle, Location, VehicleAssignment, WarehouseAssignment, RouteStep
from src.models.solution import Solution


class SolutionPresenter:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.location_by_id: Dict[str, Location] = {loc.id: loc for loc in scenario.all_locations}

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

    def _reconstruct_route(self, solver, var_manager, vehicle: Vehicle) -> List[RouteStep]:

        route_steps = []
        curr_loc_id = ""

        # 1. Находим склад отправления
        for wh in self.scenario.warehouses:
            for j_loc in self.scenario.all_locations:
                x_var = var_manager.get_routing_var(vehicle.id, wh.id, j_loc.id)
                if x_var is not None and solver.BooleanValue(x_var):
                    curr_loc_id = wh.id
                    break
            if curr_loc_id != "": break

        if curr_loc_id == "": return []

        visited_ids = set()
        max_len = len(self.scenario.all_locations) + 1

        # ДОБАВЛЯЕМ СТАРТОВЫЙ СКЛАД (Начало пути)
        route_steps.append(RouteStep(
            location=self.location_by_id[curr_loc_id],
            distance_from_prev=0.0,
            time_from_prev=0,
            delivered_volume=0,
            service_time=0
        ))

        for _ in range(max_len):
            visited_ids.add(curr_loc_id)

            next_loc_id = ""
            for j_loc in self.scenario.all_locations:
                if j_loc.id == curr_loc_id: continue
                x_var = var_manager.get_routing_var(vehicle.id, curr_loc_id, j_loc.id)
                if x_var is not None and solver.BooleanValue(x_var):
                    next_loc_id = j_loc.id
                    break

            if next_loc_id == "": break  # Тупик

            # Вытаскиваем данные перехода
            dist = self.scenario.network.distance_matrix[curr_loc_id][next_loc_id]
            time_ij = int(self.scenario.network.time_matrix[curr_loc_id][next_loc_id])

            # Вытаскиваем объем доставки в эту точку
            del_vol = 0
            for b in self.scenario.brands:
                del_var = var_manager.get_delivery_var(vehicle.id, next_loc_id, b.id)
                if del_var is not None:
                    del_vol += solver.Value(del_var)

            # Вычисляем время обслуживания
            loc_obj = self.location_by_id[next_loc_id]
            service_time = getattr(loc_obj, 'service_time', 0) if loc_obj else 0
            route_steps.append(RouteStep(
                location=self.location_by_id[next_loc_id],
                distance_from_prev=dist,
                time_from_prev=time_ij,
                delivered_volume=del_vol,
                service_time=service_time
            ))

            curr_loc_id = next_loc_id

            if curr_loc_id in [w.id for w in self.scenario.warehouses]:
                break

        return route_steps