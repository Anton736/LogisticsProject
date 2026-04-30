import math
from typing import List, Dict
from src.core.entities import Scenario, Location, Store, VehicleAssignment, WarehouseAssignment, RouteStep
from src.models.solution import Solution


class SolutionPresenter:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.location_by_id: Dict[str, Location] = {loc.id: loc for loc in scenario.all_locations}

    def build_solution(self, solution_data: dict) -> Solution:
        """
        Принимает словарь из RoutingOrchestrator:
          vehicle_routes:     {v_id: [loc_id, ...]}
          arrival_times:      {(v_id, loc_id): minutes}
          deliveries:         {(v_id, loc_id): {brand_id: units}}
          total_cost:         float
          total_revenue:      float
          optimal_lambda:     float
        """
        vehicle_routes  = solution_data['vehicle_routes']
        arrival_times   = solution_data['arrival_times']
        deliveries      = solution_data['deliveries']
        total_cost      = solution_data['total_cost']
        total_revenue   = solution_data['total_revenue']
        optimal_lambda  = solution_data.get('optimal_lambda', 0.0)

        dist_m = self.scenario.network.distance_matrix
        time_m = self.scenario.network.time_matrix
        K      = self.scenario.units_per_crate

        vehicle_assignments = []
        for v in self.scenario.vehicles:
            route_loc_ids = vehicle_routes.get(v.id, [])

            if not route_loc_ids:
                vehicle_assignments.append(VehicleAssignment(vehicle=v, route=[], is_active=False))
                continue

            steps: List[RouteStep] = []
            total_dist  = 0.0
            total_time  = 0

            for i, loc_id in enumerate(route_loc_ids):
                loc = self.location_by_id[loc_id]

                if i == 0:
                    dist_from_prev = 0.0
                    time_from_prev = 0
                else:
                    prev_id = route_loc_ids[i - 1]
                    dist_from_prev = dist_m[prev_id][loc_id]
                    time_from_prev = int(time_m[prev_id][loc_id])
                    total_dist += dist_from_prev
                    total_time += time_from_prev
                    prev_loc = self.location_by_id[prev_id]
                    if isinstance(prev_loc, Store):
                        total_time += int(prev_loc.service_time)

                # Объём доставки в эту точку (суммарно по брендам)
                store_delivery = deliveries.get((v.id, loc_id), {})
                del_vol    = sum(store_delivery.values())
                del_crates = sum(
                    math.ceil(units / K) for units in store_delivery.values()
                ) if store_delivery else 0

                service_t = int(getattr(loc, 'service_time', 0))

                steps.append(RouteStep(
                    location=loc,
                    distance_from_prev=dist_from_prev,
                    time_from_prev=time_from_prev,
                    delivered_volume=del_vol,
                    service_time=service_t,
                    delivered_crates=del_crates,
                ))

            vehicle_assignments.append(VehicleAssignment(
                vehicle=v,
                route=steps,
                total_time=total_time,
                total_dist=total_dist,
                is_active=True,
            ))

        # Склады: активен если хоть одна машина через него прошла
        active_wh_ids = {
            loc_id
            for route in vehicle_routes.values()
            for loc_id in route
            if loc_id in {wh.id for wh in self.scenario.warehouses}
        }
        warehouse_assignments = []
        for wh in self.scenario.warehouses:
            is_active = wh.id in active_wh_ids
            # Пиковый объём = суммарная доставка всех машин через этот склад
            max_vol = 0
            if is_active:
                for v in self.scenario.vehicles:
                    for (v_id, loc_id), brand_dict in deliveries.items():
                        if v_id == v.id and loc_id == wh.id:
                            max_vol += sum(math.ceil(u / K) for u in brand_dict.values())
            warehouse_assignments.append(WarehouseAssignment(
                warehouse=wh, is_active=is_active, max_volume=max_vol
            ))

        return Solution(
            vehicle_assignments=vehicle_assignments,
            warehouse_assignments=warehouse_assignments,
            optimal_objective_value=optimal_lambda,
            total_numerator_cost=total_cost,
            total_denominator_value=total_revenue,
        )