import os
from src.io.excel_mapping import ExcelMapping
from src.io.excel_loader import LogisticsExcelLoader
from src.io.excel_matrix_loader import ExcelMatrixLoader
from src.models.demand import DemandManager, DemandStep
from src.optimization.pruner import RoutePruner
from src.optimization.dinkelbach_orchestrator import DinkelbachOrchestrator
from src.core.enums import WarehouseCostMode


def main():
    import ortools
    print(ortools.__version__)
    try:
        # 1. Пути к файлам
        scenario_file   = os.path.join("data", "для логистики.xlsm")
        distances_file  = os.path.join("data", "расстояния.xlsx")
        times_file      = os.path.join("data", "время.xlsx")

        # 2. Загрузчик матриц (внешняя зависимость, легко заменить на другой источник)
        matrix_loader = ExcelMatrixLoader(
            distance_file=distances_file,
            time_file=times_file,
            distance_sheet=0,   # первый лист
            time_sheet=0,
        )

        # 3. Загрузка сценария
        mapping = ExcelMapping()
        loader = LogisticsExcelLoader(scenario_file, mapping, matrix_loader=matrix_loader)

        print("Шаг 1: Загрузка сценария из Excel...")
        scenario = loader.load_scenario()

        # 4. Вспомогательные компоненты
        demand_manager = DemandManager([DemandStep(time_limit=1440, multiplier_x100=100)])
        pruner = RoutePruner(scenario)

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
        else:
            print("Решение не найдено. Проверьте ограничения сценария.")

    except Exception as e:
        print(f"\n[КРИТИЧЕСКАЯ ОШИБКА]: {e}")


if __name__ == "__main__":
    main()