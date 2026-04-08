import os
from src.io.excel_mapping import ExcelMapping
from src.io.excel_loader import LogisticsExcelLoader
from src.io.excel_matrix_loader import ExcelMatrixLoader
from src.models.demand import DemandManager, DemandStep
from src.optimization.pruner import RoutePruner
from src.optimization.dinkelbach_orchestrator import DinkelbachOrchestrator
from src.core.enums import WarehouseCostMode
from src.io.excel_exporter import LogisticsExcelExporter


def main():
    try:
        scenario_file = os.path.join("data", "для логистики.xlsm")
        matrix_loader = ExcelMatrixLoader(
            distance_file=os.path.join("data", "matrix_distance_km.xlsx"),
            time_file=os.path.join("data", "matrix_time_minutes.xlsx")
        )

        loader = LogisticsExcelLoader(scenario_file, ExcelMapping(), matrix_loader=matrix_loader)
        scenario = loader.load_scenario()

        # 1. ПРУНЕР: Оптимальный баланс сложности
        pruner = RoutePruner(scenario, min_k_neighbors=25, max_k_neighbors=35)
        demand_manager = DemandManager([DemandStep(time_limit=1440, multiplier_x100=100)])
        print("Шаг 1: Загрузка сценария из Excel...")
        scenario = loader.load_scenario()

        # --- БЛОК АУДИТА КОСТОВ ---
        print("\n--- АУДИТ СТАВОК И КОСТОВ ---")
        if scenario.vehicles:
            v = scenario.vehicles[0]
            print(f"Ставка за выезд (Call): {v.cost_call} руб.")
            print(f"Ставка за км: {v.cost_km} руб.")
            print(f"Ставка за час: {v.cost_hour} руб.")
            print(f"Пример стоимости машины за 24 часа работы: {v.cost_call + (v.cost_hour * 24)} руб. + км")

        for wh in scenario.warehouses:
            print(f"Склад/Завод: {wh.name}, Завод: {wh.is_factory}, Фикс. кост: {wh.fixed_staff_cost}")
        print(f"Цена за единицу продукции (для выручки): {scenario.bread_unit_cost} руб.")
        print("-----------------------------\n")
        print(f"Пример дистанции (0 -> 877): {scenario.network.distance_matrix['0']['877']}")
        # 2. ОРКЕСТРАТОР: Запускаем Динкельбаха
        # Используем масштаб 100 (для учета копеек)
        orchestrator = DinkelbachOrchestrator(
            scenario=scenario,
            pruner=pruner,
            demand_manager=demand_manager,
            warehouse_cost_mode=WarehouseCostMode.PEAK_INPUT,
            objective_scale_factor=1
        )
        # --- ДИАГНОСТИКА ГРУЗА ---
        total_demand_units = sum(s.demands["B1"][1440] for s in scenario.stores)
        units_per_crate = scenario.units_per_crate
        total_demand_crates = sum(
            (s.demands["B1"][1440] + units_per_crate - 1) // units_per_crate for s in scenario.stores)
        total_fleet_capacity_crates = sum(v.capacity for v in scenario.vehicles)

        print("\n=== ДИАГНОСТИКА ГРУЗА ===")
        print(f"Общий спрос в штуках: {total_demand_units}")
        print(f"Коэффициент штук в ящике: {units_per_crate}")
        print(f"Общий спрос в ЯЩИКАХ: {total_demand_crates}")
        print(f"Общая вместимость флота (ЯЩИКИ): {total_fleet_capacity_crates}")

        if total_demand_crates > total_fleet_capacity_crates:
            print("[КРИТИЧЕСКАЯ ОШИБКА]: Твой товар не влезет в машины! Нужно больше машин или меньше товара.")

        # Проверка самого крупного магазина
        max_store = max(scenario.stores, key=lambda s: s.demands["B1"][1440])
        max_store_crates = (max_store.demands["B1"][1440] + units_per_crate - 1) // units_per_crate
        max_veh_cap = max(v.capacity for v in scenario.vehicles)

        print(f"Самый большой заказ: {max_store.name} ({max_store_crates} ящ.)")
        print(f"Самая большая машина: {max_veh_cap} ящиков")

        if max_store_crates > max_veh_cap:
            print(f"[КРИТИЧЕСКАЯ ОШИБКА]: Магазин {max_store.name} просит больше, чем влезет в ЛЮБУЮ твою машину!")
        print("==========================\n")
        print("\n=== ЗАПУСК ОПТИМИЗАЦИИ ДИНКЕЛЬБАХА (ЦЕНА / ЦЕННОСТЬ) ===")
        solution = orchestrator.solve(epsilon=0.001, max_iterations=5)

        if solution:
            print("--- Оптимизация завершена успешно ---")
            solution.print_summary()
            exporter = LogisticsExcelExporter()
            exporter.export(solution, output_path="data/маршруты_финал.xlsx")
            print("\nРезультат сохранен в data/маршруты_финал.xlsx")
        else:
            print("Решение не найдено. Попробуй еще раз уменьшить количество соседей в RoutePruner.")

    except Exception as e:
        print(f"\n[КРИТИЧЕСКАЯ ОШИБКА]: {e}")


if __name__ == "__main__":
    main()