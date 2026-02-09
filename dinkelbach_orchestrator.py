# START OF FILE dinkelbach_orchestrator.py
from ortools.sat.python import cp_model
from typing import Tuple, Optional, Dict, Any

from entities import Scenario
from var_manager import VarManager
from RouterPruner import RoutePruner
from DemandStep import DemandManager
from constraints import ConstraintFactory
from objective_builder import ObjectiveBuilder
from enums import WarehouseCostMode
from solution import Solution  # Будет создан ниже
from solution_presenter import SolutionPresenter  # Будет создан ниже

import math


class DinkelbachOrchestrator:
    def __init__(self,
                 scenario: Scenario,
                 pruner: RoutePruner,
                 demand_manager: DemandManager,
                 warehouse_cost_mode: WarehouseCostMode = WarehouseCostMode.PEAK_INPUT,
                 objective_scale_factor: int = 1000):  # Для ObjBuilder

        self.scenario = scenario
        self.pruner = pruner
        self.demand_manager = demand_manager
        self.warehouse_cost_mode = warehouse_cost_mode
        self.objective_scale_factor = objective_scale_factor  # Множитель для float в int в ObjectiveBuilder
        self.solver = cp_model.CpSolver()

        # Настройки решателя (можно вынести в конфигурацию)
        self.solver.parameters.max_time_in_seconds = 600.0  # 10 минут, для больших задач может быть увеличено
        self.solver.parameters.num_workers = 8  # Использовать 8 ядер для поиска решения
        self.solver.parameters.log_search_progress = True  # Выводить логи прогресса
        self.solver.parameters.random_seed = 42  # Для воспроизводимости результатов (если возможно)

    def solve(self, epsilon: float = 1e-6, max_iterations: int = 100) -> Optional[Solution]:
        """
        Реализует алгоритм Динкельбаха для решения дробно-линейной оптимизации.
        Возвращает объект Solution, если найдено оптимальное решение.
        """
        best_lambda = 0.0  # Начальное значение лямбда
        best_solution_info: Optional[Dict[str, Any]] = None  # Храним информацию о лучшем решении

        # Масштабирующий фактор для лямбды, чтобы она была int при умножении на denominator_expr
        # Если OBJ_SCALE = 1000 (3 знака), а нам нужна лямбда с 6-8 знаками, то 10^9 или 10^12
        LAMBDA_SCALE_FACTOR = 1_000_000_000  # 9 знаков после запятой для лямбда

        print(f"Starting Dinkelbach algorithm with epsilon={epsilon}, max_iterations={max_iterations}")
        print(f"Objective scaling factor: {self.objective_scale_factor}, Lambda scaling factor: {LAMBDA_SCALE_FACTOR}")

        for iteration in range(max_iterations):
            print(f"\n--- Dinkelbach Iteration {iteration + 1}, Current Lambda: {best_lambda:.8f} ---")

            # Каждая итерация Динкельбаха требует нового экземпляра модели CP-SAT
            model = cp_model.CpModel()
            var_manager = VarManager(model, self.scenario, self.pruner)

            # Добавляем все ограничения
            constraint_factory = ConstraintFactory(
                model, self.scenario, var_manager, self.demand_manager, self.pruner,
                warehouse_cost_mode=self.warehouse_cost_mode  # Передаем выбранный режим
            )
            constraint_factory.add_all_constraints()

            # Строим выражения для числителя и знаменателя
            objective_builder = ObjectiveBuilder(
                model, self.scenario, var_manager, scale_factor=self.objective_scale_factor
            )
            numerator_expr, denominator_expr = objective_builder.build_objective_expressions()

            # Формируем целевую функцию для текущей итерации Динкельбаха: Minimize (Numerator - lambda * Denominator)
            # Все выражения уже масштабированы на `self.objective_scale_factor`.
            # Теперь масштабируем всю целевую функцию на `LAMBDA_SCALE_FACTOR` для точности `lambda`.
            scaled_lambda_int = int(best_lambda * LAMBDA_SCALE_FACTOR)

            # Целевая функция: (Numerator * LAMBDA_SCALE_FACTOR) - (scaled_lambda_int * Denominator)
            # Обе части выражения теперь целочисленные
            objective_to_minimize = (numerator_expr * LAMBDA_SCALE_FACTOR) - (scaled_lambda_int * denominator_expr)
            model.Minimize(objective_to_minimize)

            # Решаем модель
            status = self.solver.Solve(model)

            if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
                current_numerator_value_scaled = self.solver.Value(numerator_expr)
                current_denominator_value_scaled = self.solver.Value(denominator_expr)

                # Чтобы получить реальные, немасштабированные значения для расчета лямбды
                # (и для вывода в SolutionPresenter)
                current_numerator_value_real = current_numerator_value_scaled / self.objective_scale_factor
                current_denominator_value_real = current_denominator_value_scaled / self.objective_scale_factor

                if current_denominator_value_real == 0:
                    # Если знаменатель 0, это означает, что ничего не доставлено.
                    # Если числитель тоже 0, то lambda=0 (идеально, но нереалистично).
                    # Если числитель > 0, то дробь стремится к бесконечности.
                    # В контексте VRP с дробной целью, 0 доставленных товаров обычно неоптимально,
                    # если есть спрос и стоимость. Это может указывать на то, что
                    # любая доставка слишком дорога.
                    if current_numerator_value_real == 0:
                        new_lambda = 0.0
                        print(f"Iteration {iteration + 1}: Real Numerator = 0, Real Denominator = 0. New lambda = 0.")
                    else:
                        print(
                            f"Iteration {iteration + 1}: Real Numerator = {current_numerator_value_real:.2f}, Real Denominator = 0. "
                            "Cannot compute new lambda (divide by zero). This may indicate no cost-effective delivery is possible.")
                        # Если на первом проходе (lambda=0) мы получаем 0 в знаменателе,
                        # это означает, что даже бесплатная доставка невозможна или невыгодна.
                        if iteration == 0 and current_numerator_value_real > 0:
                            print("Problem might be fundamentally infeasible or optimal solution.py is zero delivery.")
                            return None  # Или break, если хотим вернуть предыдущее лучшее решение

                        # Если это не первый проход, и мы получили 0 в знаменателе,
                        # это значит, что мы пытаемся снизить лямбду настолько, что доставка
                        # становится нерентабельной или невозможной.
                        # В таком случае, предыдущая лямбда была "лучше".
                        print("Converged to a solution.py with zero denominator. Using previous best lambda.")
                        break  # Считаем, что достигли предела.
                else:
                    new_lambda = current_numerator_value_real / current_denominator_value_real

                print(
                    f"Iteration {iteration + 1} results: Real Numerator = {current_numerator_value_real:.2f}, Real Denominator = {current_denominator_value_real:.2f}")
                print(f"New lambda calculated: {new_lambda:.8f}")

                if abs(new_lambda - best_lambda) < epsilon:
                    print(f"Dinkelbach converged! Optimal lambda: {new_lambda:.8f}")
                    best_solution_info = {
                        "solver": self.solver,
                        "var_manager": var_manager,
                        "numerator_value_scaled": current_numerator_value_scaled,
                        "denominator_value_scaled": current_denominator_value_scaled,
                        "optimal_lambda": new_lambda,
                        "objective_scale_factor": self.objective_scale_factor  # Сохраняем для Presenter
                    }
                    break  # Алгоритм сошелся
                else:
                    best_lambda = new_lambda
                    # На каждом шаге сохраняем лучшее *найденное* решение, пока оно не сойдется
                    best_solution_info = {
                        "solver": self.solver,
                        "var_manager": var_manager,
                        "numerator_value_scaled": current_numerator_value_scaled,
                        "denominator_value_scaled": current_denominator_value_scaled,
                        "optimal_lambda": new_lambda,
                        "objective_scale_factor": self.objective_scale_factor
                    }
            elif status == cp_model.INFEASIBLE:
                print(f"Dinkelbach Iteration {iteration + 1}: Model is INFEASIBLE.")
                if iteration == 0:
                    print("Problem is fundamentally infeasible from the start.")
                    return None
                else:
                    print(
                        "Model became infeasible. This can happen if lambda is too high. Reverting to previous best solution.py.")
                    break
            elif status == cp_model.ABORTED:
                print(f"Dinkelbach Iteration {iteration + 1}: Model ABORTED (e.g., timeout or user interrupt).")
                break
            else:
                print(f"Dinkelbach Iteration {iteration + 1}: Unknown solver status {self.solver.StatusName(status)}.")
                break

        if best_solution_info:
            print("\n--- Building Final Solution Object ---")
            presenter = SolutionPresenter(self.scenario)
            final_solution = presenter.build_solution(
                solver=best_solution_info["solver"],
                var_manager=best_solution_info["var_manager"],
                optimal_lambda=best_solution_info["optimal_lambda"],
                numerator_value_scaled=best_solution_info["numerator_value_scaled"],
                denominator_value_scaled=best_solution_info["denominator_value_scaled"],
                objective_scale_factor=best_solution_info["objective_scale_factor"]
            )
            return final_solution
        else:
            print("No feasible or optimal solution.py found after Dinkelbach iterations.")
            return None