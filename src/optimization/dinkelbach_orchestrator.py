from ortools.sat.python import cp_model
from src.optimization.var_manager import VarManager
from src.optimization.constraints import ConstraintFactory
from src.optimization.objective_builder import ObjectiveBuilder
from src.io.solution_presenter import SolutionPresenter


class DinkelbachOrchestrator:
    def __init__(self, scenario, pruner, demand_manager, warehouse_cost_mode, objective_scale_factor=1):
        self.scenario = scenario
        self.pruner = pruner
        self.demand_manager = demand_manager
        self.warehouse_cost_mode = warehouse_cost_mode
        self.objective_scale_factor = objective_scale_factor
        self.solver = cp_model.CpSolver()
        # Ставим 2 минуты на итерацию
        self.solver.parameters.max_time_in_seconds = 120.0
        self.solver.parameters.log_search_progress = True

    def solve(self, epsilon=0.01, max_iterations=3):
        current_lambda = 0.0
        best_sol = None
        # Масштаб для дробного коэффициента (лямбды)
        L_SCALE = 1

        for iteration in range(max_iterations):
            print(f"\n>>> ИТЕРАЦИЯ ДИНКЕЛЬБАХА {iteration + 1} (Lambda: {current_lambda:.4f})")

            model = cp_model.CpModel()
            var_manager = VarManager(model, self.scenario, self.pruner)
            factory = ConstraintFactory(model, self.scenario, var_manager, self.demand_manager, self.pruner)

            # Добавляем все ограничения
            factory.add_all_constraints()

            # Строим выражения числителя (затраты) и знаменателя (выручка)
            obj_builder = ObjectiveBuilder(model, self.scenario, var_manager, scale_factor=self.objective_scale_factor)
            num_expr, den_expr = obj_builder.build_objective_expressions()

            # Целевая функция: Costs * L_SCALE - Value * current_lambda_int
            l_int = int(current_lambda * L_SCALE)
            model.minimize(num_expr * L_SCALE - den_expr * l_int)

            # Решаем
            status = self.solver.Solve(model)

            # ПРОВЕРКА: Есть ли хоть какое-то решение? (Даже если UNKNOWN)
            has_response = False
            try:
                # Пытаемся получить значение целевой функции. Если оно есть - решение в памяти
                _ = self.solver.ObjectiveValue()
                has_response = True
            except:
                has_response = False

            if has_response:
                print(f"Итерация {iteration + 1}: Статус {self.solver.StatusName(status)}, решение найдено!")

                v_num = self.solver.Value(num_expr)
                v_den = self.solver.Value(den_expr)

                # Считаем реальные значения (без масштабов)
                real_num = v_num / self.objective_scale_factor
                real_den = v_den / self.objective_scale_factor

                # Новая лямбда (эффективность)
                new_lambda = real_num / max(1.0, real_den)
                new_lambda = min(new_lambda, 1000.0)  # <--- ДОБАВЬ ЭТО
                print(f"Итог: Затраты={real_num:.2f}, Выручка={real_den:.2f}, Эффективность={new_lambda:.6f}")

                # Создаем объект решения
                presenter = SolutionPresenter(self.scenario)
                best_sol = presenter.build_solution(
                    self.solver, var_manager, new_lambda, v_num, v_den, self.objective_scale_factor
                )

                if abs(new_lambda - current_lambda) < epsilon:
                    print("Алгоритм сошелся по точности.")
                    break
                current_lambda = new_lambda
            else:
                print(f"Итерация {iteration + 1} завершилась без решения (Статус: {self.solver.StatusName(status)})")
                break

        return best_sol