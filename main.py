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
        # =========================================================

        # 4. Вспомогательные компоненты
        demand_manager = DemandManager([DemandStep(time_limit=1440, multiplier_x100=100)])
        pruner = RoutePruner(scenario, min_k_neighbors=15,max_k_neighbors=25)

        # 5. Оптимизация
        print("Шаг 2: Запуск оптимизации (Алгоритм Динкельбаха)...")
        orchestrator = DinkelbachOrchestrator(
            scenario=scenario,
            pruner=pruner,
            demand_manager=demand_manager,
            warehouse_cost_mode=WarehouseCostMode.PEAK_INPUT,
        )
        solution = orchestrator.solve()

        # 6. Результаты
        if solution:
            print("--- Оптимизация завершена успешно ---")
            solution.print_summary()
            exporter = LogisticsExcelExporter()
            exporter.export(solution, output_path="data/маршруты_результат.xlsx")
        else:
            print("Решение не найдено. Проверьте ограничения сценария.")

    except Exception as e:
        print(f"\n[КРИТИЧЕСКАЯ ОШИБКА]: {e}")


if __name__ == "__main__":
    main()