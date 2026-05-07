import os
from src.io.excel_mapping import ExcelMapping
from src.io.excel_loader import LogisticsExcelLoader
from src.io.excel_matrix_loader import ExcelMatrixLoader
from src.models.demand import DemandManager, DemandStep
from src.optimization.fleet_optimizer import FleetOptimizer
from src.optimization.pruner import RoutePruner
from src.core.enums import WarehouseCostMode
from src.io.excel_exporter import LogisticsExcelExporter
from src.optimization.routing_orchestrator import RoutingOrchestrator
from src.optimization.time_revenue_manager import LinearTimeRevenueManager
from map_osrm import build_route_map

def main(path):
    try:
        # 1. Пути к файлам
        scenario_file   = os.path.join("data", "для логистики.xlsm")
        distances_file  = os.path.join("data", "matrix_distance_km.xlsx")
        times_file      = os.path.join("data", "matrix_time_minutes.xlsx")

        # 2. Загрузчик матриц
        matrix_loader = ExcelMatrixLoader(
            distance_file=distances_file,
            time_file=times_file,
            distance_sheet=0,
            time_sheet=0,
        )

        # 3. Загрузка сценария
        mapping = ExcelMapping()
        loader = LogisticsExcelLoader(scenario_file, mapping, matrix_loader=matrix_loader)

        print("Шаг 1: Загрузка сценария из Excel...")
        scenario = loader.load_scenario()

        # =========================================================
        # НОВЫЙ БЛОК: ПРОВЕРКА ЗАГРУЖЕННЫХ ДАННЫХ
        # =========================================================
        print("\n--- Отчет о загрузке данных ---")
        print(f"Магазинов (Stores): {len(scenario.stores)}")
        print(f"Складов (Warehouses): {len(scenario.warehouses)}")
        print(f"Машин (Vehicles): {len(scenario.vehicles)}")
        total_demand_units = sum(store.demands["B1"][1440] for store in scenario.stores)
        total_capacity_crates = sum(v.capacity for v in scenario.vehicles)
        total_capacity_units = total_capacity_crates * scenario.units_per_crate

        print(f"Общий заказ: {total_demand_units} ЕДИНИЦ продукции.")
        print(f"Вместимость: {total_capacity_crates} ЯЩИКОВ (или {total_capacity_units} единиц).")
        print("-------------------------------\n")

        if len(scenario.stores) == 0:
            print("[СТОП] Загружено 0 магазинов. Оптимизация невозможна.")
            print("Возможные причины:")
            print(" 1. В Excel столбец называется не '№' (возможно есть невидимый пробел).")
            print(" 2. В столбце '№' нет чисел (только текст), фильтр их удалил.")
            return

        if len(scenario.vehicles) == 0:
            print("[СТОП] Загружено 0 машин. Оптимизация невозможна.")
            print("Причина: Не найдены данные о машинах в справочнике.")
            return

        # --- ДИАГНОСТИКА ФЛОТА ---
        units_per_crate = scenario.units_per_crate
        total_demand_crates = sum(
            (s.demands["B1"][1440] + units_per_crate - 1) // units_per_crate for s in scenario.stores)
        total_fleet_capacity_crates = sum(v.capacity for v in scenario.vehicles)
        max_store = max(scenario.stores, key=lambda s: s.demands["B1"][1440])
        max_store_crates = (max_store.demands["B1"][1440] + units_per_crate - 1) // units_per_crate
        max_veh_cap = max(v.capacity for v in scenario.vehicles)

        print(f"Общий спрос в ящиках: {total_demand_crates}, вместимость флота: {total_fleet_capacity_crates} ящ.")
        if total_demand_crates > total_fleet_capacity_crates:
            print("[КРИТИЧЕСКАЯ ОШИБКА]: Товар не влезет в машины! Нужно больше машин или меньше товара.")
            return
        if max_store_crates > max_veh_cap:
            print(f"[КРИТИЧЕСКАЯ ОШИБКА]: Магазин {max_store.name} ({max_store_crates} ящ.) превышает вместимость любой машины ({max_veh_cap} ящ.).")
            return
        # =========================================================

        # 4. Вспомогательные компоненты
        demand_manager = DemandManager([DemandStep(time_limit=1440, multiplier_x100=100)])
        pruner = RoutePruner(scenario, min_k_neighbors=60, max_k_neighbors=75)
        time_manager = LinearTimeRevenueManager(drop_to_percent=0.0)
        # 5. Оптимизация
        # 5. Оптимизация
        print("Шаг 2: Разведочный запуск для определения базы (ищем 60-120 секунд)...")

        # 5.1 Разведочный запуск (нужно чтобы в Orchestrator был добавлен параметр time_limit_seconds)
        scout_orchestrator = RoutingOrchestrator(
            scenario=scenario,
            pruner=pruner,
            demand_manager=demand_manager,
            warehouse_cost_mode=WarehouseCostMode.PEAK_INPUT,
            time_manager=time_manager,
            time_limit_per_run=60  #<-- Раскомментируйте, когда добавите этот параметр в Orchestrator
        )

        # Ставим max_iterations=2 или 3, чтобы Динкельбах быстро завершился
        scout_solution = scout_orchestrator.solve(max_iterations=2)

        # 5.2 Определяем базу
        # 5.2 Определяем базу
        if scout_solution and scout_solution.vehicle_assignments:
            # Считаем только те машины, которые реально поехали (is_active)
            active_vehicles = [va for va in scout_solution.vehicle_assignments if va.is_active]
            base_vehicles = len(active_vehicles)

            if base_vehicles == 0:
                base_vehicles = len(scenario.vehicles)  # На всякий случай
            print(f"Разведочный запуск решил использовать {base_vehicles} машин.")
        else:
            print("Разведочный запуск не нашел быстрого решения. Берем все машины.")
            base_vehicles = len(scenario.vehicles) - 2  # Фолбэк

        # 5.3 Формируем окно +-2 машины
        min_v = max(1, base_vehicles - 2)
        max_v = min(len(scenario.vehicles), base_vehicles + 2)

        print(f"Шаг 3: Запускаем точный поиск в окне от {min_v} до {max_v} машин...")

        # 5.4 Финальный перебор флота
        optimizer = FleetOptimizer(
            scenario=scenario,
            pruner=pruner,
            demand_manager=demand_manager,
            warehouse_cost_mode=WarehouseCostMode.PEAK_INPUT,
            time_manager=time_manager,
            min_vehicles=min_v,
            max_vehicles=max_v,
            time_limit_per_run=60,  # 60 секунд на каждый вариант
        )

        solution = optimizer.optimize()

        # 6. Результаты
        if solution:
            print("--- Оптимизация завершена успешно ---")
            solution.print_summary()
            exporter = LogisticsExcelExporter()
            exporter.export(solution, output_path=path)
        else:
            print("Решение не найдено. Проверьте ограничения сценария.")

    except Exception as e:
        print(f"\n[КРИТИЧЕСКАЯ ОШИБКА]: {e}")


if __name__ == "__main__":
    str = "data/маршруты_результат.xlsx"
    main(str)
    build_route_map(routes_excel_path=str)