from dataclasses import dataclass
from typing import List
from ortools.sat.python import cp_model


@dataclass(frozen=True)
class DemandStep:
    time_limit: int  # До какого времени (в минутах) действует интервал
    multiplier_x100: int  # Коэффициент, умноженный на 100 (например, 110 для +10%)


class DemandManager:
    def __init__(self, steps: List[DemandStep]):
        # Сортируем шаги по времени для корректной логики интервалов
        self.steps = sorted(steps, key=lambda x: x.time_limit)

    def get_max_demand_at_time(self, model: cp_model.CpModel, arrival_time: cp_model.IntVar,
                               base_demand: int) -> cp_model.IntVar:
        """
        Возвращает переменную, ограничивающую спрос в зависимости от времени прибытия.
        Реализует кусочно-постоянную функцию спроса.
        """
        # Переменная для итогового максимально допустимого объема (n_it'vb)
        max_vol_at_t = model.NewIntVar(0, int(base_demand * 1.5), f"max_vol_at_{arrival_time}")

        # Создаем булевы переменные для каждого временного интервала
        segments = []
        for i, step in enumerate(self.steps):
            is_in_segment = model.NewBoolVar(f"segment_{i}_at_{step.time_limit}")
            segments.append(is_in_segment)

            # Определяем границы интервала
            lower_limit = self.steps[i - 1].time_limit if i > 0 else 0

            # Связываем время с интервалом: lower_limit <= arrival_time < step.time_limit
            # Примечание: Это упрощенная логика, для полной нужны индикаторы входа
            model.AddLinearConstraint(arrival_time, lower_limit, step.time_limit - 1).OnlyEnforceIf(is_in_segment)

            # Если мы в этом интервале, фиксируем max_vol
            # n_it'vb = (base_demand * multiplier) / 100
            calculated_demand = (base_demand * step.multiplier_x100) // 100
            model.Add(max_vol_at_t == calculated_demand).OnlyEnforceIf(is_in_segment)

        # Ограничение: ровно один сегмент должен быть активен
        model.Add(sum(segments) == 1)

        return max_vol_at_t