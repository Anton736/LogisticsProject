# START OF FILE solution.py
from dataclasses import dataclass
from typing import List

from entities import VehicleAssignment, WarehouseAssignment


@dataclass
class Solution:
    """Класс для хранения финального решения оптимизации."""
    vehicle_assignments: List[VehicleAssignment]
    warehouse_assignments: List[WarehouseAssignment]
    optimal_objective_value: float  # Оптимальное значение Cost/Value (лямбда*)
    total_numerator_cost: float  # Общие затраты (немасштабированные)
    total_denominator_value: float  # Общая ценность доставленного товара (немасштабированная)

    def print_summary(self):
        print("\n--- Final Solution Summary ---")
        print(f"Optimal Fractional Objective (Cost/Value): {self.optimal_objective_value:.8f}")
        print(f"Total Costs: {self.total_numerator_cost:,.2f}")
        print(f"Total Value of Delivered Goods: {self.total_denominator_value:,.2f}")

        print("\n--- Vehicle Assignments ---")
        active_vehicles = [va for va in self.vehicle_assignments if va.is_active]
        if active_vehicles:
            for va in active_vehicles:
                print(f"Vehicle {va.vehicle.id} (Category: {va.vehicle.category}):")
                route_names = [f"{loc.name} (ID: {loc.id})" for loc in va.route] if va.route else ["No route"]
                print(f"  Route: {' -> '.join(route_names)}")
                print(f"  Total Distance: {va.total_dist:,.2f} km")
                print(f"  Total Time: {va.total_time:,.2f} min")
                print(f"  Cost: {va.get_total_cost():,.2f}")
        else:
            print("No vehicles were used.")

        print("\n--- Warehouse Assignments ---")
        active_warehouses = [wa for wa in self.warehouse_assignments if wa.is_active]
        if active_warehouses:
            for wa in active_warehouses:
                print(f"Warehouse {wa.warehouse.id} ({wa.warehouse.name}):")
                print(f"  Max Volume Handled (w_iv): {wa.max_volume:,.0f}")
                print(f"  Cost: {wa.get_warehouse_cost():,.2f}")
        else:
            print("No warehouses were used.")