import os
from src.io.excel_mapping import ExcelMapping
from src.io.excel_loader import LogisticsExcelLoader
from src.models.demand import DemandManager, DemandStep
from src.optimization.pruner import RoutePruner
from src.optimization.dinkelbach_orchestrator import DinkelbachOrchestrator
from src.core.enums import WarehouseCostMode


def main():
    try:
        # 1. Настройка путей
        # Убедись, что файл лежит в папке data/
        file_path = os.path.join("data", "для логистики.xlsm")

        # 2. Инициализация загрузчика (Паттерн Адаптер)
        mapping = ExcelMapping()
        loader = LogisticsExcelLoader(file_path, mapping)

        print("Шаг 1: Загрузка сценария из Excel...")
        scenario = loader.load_scenario()

        # 3. Настройка вспомогательных компонентов
        # Пока ставим один шаг спроса (100% на весь день)
        demand_manager = DemandManager([DemandStep(time_limit=1440, multiplier_x100=100)])

        # Обрезка графа (ускоряет расчет в десятки раз)
        pruner = RoutePruner(scenario)

        # 4. Сборка и запуск оркестратора (Паттерн Стратегия/Фасад)
        print("Шаг 2: Запуск оптимизации (Алгоритм Динкельбаха)...")
        orchestrator = DinkelbachOrchestrator(
            scenario=scenario,
            pruner=pruner,
            demand_manager=demand_manager,
            warehouse_cost_mode=WarehouseCostMode.PEAK_INPUT
        )

        # В dinkelbach_orchestrator.py оберни цикл итераций в метод run()
        solution = orchestrator.solve()

        # 5. Вывод результатов
        if solution:
            print("--- Оптимизация завершена успешно ---")
            solution.print_summary()
        else:
            print("!!! Решение не найдено. Проверьте ограничения сценария.")
    except Exception as e:
        print(f"\n[КРИТИЧЕСКАЯ ОШИБКА]: {e}")

if __name__ == "__main__":
    main()